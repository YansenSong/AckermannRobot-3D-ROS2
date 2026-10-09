import React, { useEffect, useRef, useState } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { useRos, useRosStatus } from "../app/App";
import AreaRulesEditor from "../components/AreaRulesEditor";
import { AppConfig } from "../shared/constants";
import {
  DashboardCard,
  EmptyState,
  SectionHeader,
  StatusBadge,
} from "../shared/ui/Dashboard";
import { useT } from "../shared/i18n/i18n";
import { useRoleAccess } from "../shared/auth/roleAccess";

// 传输时用下划线代替空格；UI 中显示为空格。
const toWire = (s) =>
  String(s || "")
    .trim()
    .replace(/\s+/g, "_");
const toDisplay = (s) => String(s || "").replace(/_/g, " ");
const stripCsv = (s) => String(s || "").replace(".csv", "");

// 数据结构：[ { group: [ { map: [route,...] }, ... ] }, ... ]。
const parseStructure = (structure) =>
  (structure || []).map((groupObj) => {
    const group = Object.keys(groupObj)[0];
    const maps = (groupObj[group] || []).map((mapObj) => {
      const map = Object.keys(mapObj)[0];
      return {
        name: toDisplay(map),
        routes: (mapObj[map] || []).map((r) => toDisplay(stripCsv(r))),
      };
    });
    return { name: toDisplay(group), maps };
  });

const inputClass =
  "rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover outline-none focus:border-themeBlue";

const matchesActiveMap = (active, selection) =>
  selection?.source === "project"
    ? active.projectMap === selection.map
    : !active.projectMap &&
      active.group === selection?.group &&
      active.map === selection?.map;

/**
 * 地图管理：保存当前地图、切换已保存地图、重命名和删除地图，并将地图整理到分组中。通过 /ui_operation 与现有
 * folders_handler 后端通信（使用与 Routes 页面相同的命令协议），并从 /nav_data_resp 读取目录。
 */
const MapsPage = () => {
  const { t } = useT();
  const mapAccess = useRoleAccess("Engineer");
  const ros = useRos();
  const rosStatus = useRosStatus();
  const connected = rosStatus === "connected";

  const [groups, setGroups] = useState([]);
  const [projectMaps, setProjectMaps] = useState([]);
  const [active, setActive] = useState({ group: "", map: "", projectMap: "" });
  const [renaming, setRenaming] = useState(null); // {group, map, value}
  const [editingMap, setEditingMap] = useState(null);
  const [pendingEdit, setPendingEdit] = useState(null);

  const reqRef = useRef(null);
  const respRef = useRef(null);
  const opRef = useRef(null);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return undefined;

    reqRef.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.NAV_DATA_REQ_TOPIC,
      messageType: "std_msgs/Empty",
    });
    respRef.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.NAV_DATA_RESP_TOPIC,
      messageType: "std_msgs/String",
    });
    opRef.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.UI_OPERATION_TOPIC,
      messageType: "std_msgs/String",
    });

    respRef.current.subscribe((data) => {
      try {
        const obj = JSON.parse(data.data);
        if (obj.catalog_source !== "maps") return;
        setGroups(parseStructure(obj.structure));
        setProjectMaps(Array.isArray(obj.project_maps) ? obj.project_maps : []);
        setActive({
          group: toDisplay(obj.active_files?.group),
          map: toDisplay(obj.active_files?.map),
          projectMap: obj.active_files?.project_map || "",
        });
      } catch {
        // nav_data 格式错误，忽略此消息。
      }
    });

    reqRef.current.publish();
    return () => respRef.current?.unsubscribe();
  }, [ros]);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return undefined;
    const topic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.UI_MESSAGE_TOPIC,
      messageType: "std_msgs/String",
    });
    const handler = (message) => {
      const text = String(message?.data || "");
      if (text.startsWith('Map loaded "')) {
        toast.success(text);
      } else if (
        text.startsWith('Created group "') ||
        text.startsWith('Renamed map "')
      ) {
        toast.success(text);
      } else if (
        text.startsWith("Map switch failed:") ||
        text.startsWith("Map switch rejected:") ||
        text.startsWith("MapServer loaded the 2D map, but saving or refreshing") ||
        text.startsWith("Map rename failed:") ||
        text.startsWith("Group creation failed:") ||
        text.startsWith("Group rename failed:")
      ) {
        toast.error(text);
      } else if (text.startsWith('Deleted map "') && text.endsWith('"')) {
        const name = text.slice('Deleted map "'.length, -1);
        toast.success(t('Deleted "{name}"').replace("{name}", name));
      } else if (text.startsWith('Deleted group "') && text.endsWith('"')) {
        const name = text.slice('Deleted group "'.length, -1);
        toast.success(t('Deleted group "{name}"').replace("{name}", name));
      } else if (
        text.startsWith("Map deletion failed:") ||
        text.startsWith("Group deletion failed:")
      ) {
        toast.error(text);
      }
    };
    topic.subscribe(handler);
    return () => topic.unsubscribe(handler);
  }, [ros, t]);

  const refresh = () => reqRef.current?.publish();

  const sendCmd = (path, data, preserveNames = false) => {
    if (!mapAccess.allowed) {
      toast.warn(t(mapAccess.reason));
      return false;
    }
    if (!opRef.current) return false;
    const wired = data && !preserveNames
      ? Object.fromEntries(
          Object.entries(data).map(([k, v]) => [
            k,
            typeof v === "string" ? toWire(v) : v,
          ]),
        )
      : data;
    const msg = wired ? `${path}/${JSON.stringify(wired)}` : path;
    opRef.current.publish(new window.ROSLIB.Message({ data: msg }));
    // 后端完成文件操作后会重新发布 nav_data；此处也主动触发一次刷新。
    setTimeout(refresh, 900);
    return true;
  };

  const switchMap = (group, map) => {
    if (sendCmd("change_map", { group, map })) {
      toast.info(`正在加载 2D 地图“${map}”；LIORF 点云先验不会随之切换。`);
    }
  };

  const switchProjectMap = (map) => {
    if (sendCmd("change_project_map", { map }, true)) {
      toast.info(`正在加载 2D 地图“${map}”；LIORF 点云先验不会随之切换。`);
    }
  };

  const editMap = (group, map, source = "ui") => {
    if (!mapAccess.allowed) {
      toast.warn(t(mapAccess.reason));
      return;
    }
    if (!connected) return;
    const selection = { group, map, source };
    if (matchesActiveMap(active, selection)) {
      setEditingMap(selection);
      return;
    }
    setPendingEdit(selection);
    if (source === "project") switchProjectMap(map);
    else switchMap(group, map);
  };

  useEffect(() => {
    if (!pendingEdit || !matchesActiveMap(active, pendingEdit)) return;
    setEditingMap(pendingEdit);
    setPendingEdit(null);
  }, [active, pendingEdit]);

  useEffect(() => {
    if (!pendingEdit) return undefined;
    const timer = window.setTimeout(() => {
      setPendingEdit(null);
      toast.error("地图未能在 15 秒内加载，请检查机器人状态后重试");
    }, 15000);
    return () => window.clearTimeout(timer);
  }, [pendingEdit]);

  useEffect(() => {
    if (editingMap && active.group &&
      !matchesActiveMap(active, editingMap)) {
      setEditingMap(null);
    }
  }, [active, editingMap]);

  const deleteMap = (group, map) => {
    if (
      !window.confirm(
        t('Delete map "{map}" and its routes? This cannot be undone.').replace(
          "{map}",
          map,
        ),
      )
    )
      return;
    if (sendCmd("delete_map", { group, map })) {
      toast.info(t('Deleting "{name}"…').replace("{name}", map));
    }
  };

  const commitRename = () => {
    if (!renaming) return;
    const next = renaming.value.trim();
    if (next && next !== renaming.map) {
      const sent = sendCmd("rename_map", {
        group: renaming.group,
        map_old: renaming.map,
        map_new: next,
        active:
          active.group === renaming.group && active.map === renaming.map,
      });
      if (sent) toast.info(t('Renaming to "{name}"…').replace("{name}", next));
    }
    setRenaming(null);
  };

  const deleteGroup = (group) => {
    if (
      !window.confirm(
        t('Delete group "{name}" and everything in it?').replace(
          "{name}",
          group,
        ),
      )
    )
      return;
    if (sendCmd("delete_group", { group })) {
      toast.info(t('Deleting group "{name}"…').replace("{name}", group));
    }
  };

  if (editingMap) {
    return (
      <div className="sectionHeight space-y-4 py-4 sm:py-6">
        <ToastContainer position="bottom-right" theme="dark" />
        <button onClick={() => setEditingMap(null)}
          className="rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeBlue hover:border-themeBlue">
          ← 返回已保存的地图
        </button>
        <SectionHeader eyebrow="Map rules" title={`编辑地图：${editingMap.map}`}
          description={editingMap.source === "project"
            ? `位置：maps/${editingMap.map} · 当前加载地图的规则将在机器人侧保存。`
            : `分组：${editingMap.group} · 当前加载地图的规则将在机器人侧保存。`} />
        <DashboardCard className="p-4"><AreaRulesEditor /></DashboardCard>
      </div>
    );
  }

  return (
    <div className="sectionHeight space-y-5 py-4 sm:py-6">
      <ToastContainer position="bottom-right" theme="dark" />
      <SectionHeader
        eyebrow="Environments"
        title="Maps"
        description="管理已保存地图。切换按钮仅重载 2D map_server；完整导航地图需要重启 LIORF。"
        action={
          <div className="flex items-center gap-3">
            <StatusBadge
              status={connected ? "connected" : "disconnected"}
              pulse={connected}
              label={connected ? "live" : "offline"}
            />
            <button
              onClick={refresh}
              disabled={!connected}
              className="rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeTextGray hover:border-themeBlue hover:text-themeBlue disabled:opacity-40"
            >
              {t("Refresh")}
            </button>
          </div>
        }
      />

      <DashboardCard className="p-4">
        <div className="mb-3">
          <p className="text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
            {t("Saved maps")}
          </p>
        </div>

        {groups.length === 0 && projectMaps.length === 0 ? (
          <EmptyState
            title="No maps yet"
            description="在 maps/ 下添加包含 map.yaml 和地图图像的目录，然后点击刷新。"
          />
        ) : (
          <div className="space-y-4">
            {projectMaps.length > 0 && (
              <div>
                <p className="mb-1.5 font-[RobotoMono] text-xs uppercase tracking-wider text-themeTextGray">
                  maps/ 目录
                </p>
                <div className="space-y-1.5">
                  {projectMaps.map((m) => {
                    const isActive = active.projectMap === m.name;
                    return (
                      <div
                        key={m.name}
                        className={`flex flex-wrap items-center gap-3 rounded-lg border px-3 py-2 ${
                          isActive
                            ? "border-themeBlue/50 bg-themeBlue/5"
                            : "border-borderSubtle bg-bgSurface"
                        }`}
                      >
                        <span className="min-w-0 flex-1 truncate text-sm font-semibold text-textWhiteHover">
                          {m.name}
                          {isActive && (
                            <span className="ml-2 rounded border border-themeBlue/40 px-1.5 py-0.5 text-[10px] text-themeBlue">
                              {t("active")}
                            </span>
                          )}
                          {!m.loadable && (
                            <span className="ml-2 text-[11px] font-normal text-statusYellow">
                              尚未生成可加载的 2D 地图
                            </span>
                          )}
                        </span>
                        <div className="flex shrink-0 items-center gap-1.5 font-[RobotoMono] text-xs">
                          <button
                            onClick={() => switchProjectMap(m.name)}
                            disabled={!connected || !m.loadable || isActive}
                            className="rounded-lg border border-themeBlue px-2 py-1 text-themeBlue hover:bg-themeBlue hover:text-white disabled:opacity-40"
                          >
                            {t(isActive ? "Loaded" : "Switch")}
                          </button>
                          <button
                            onClick={() => editMap("maps", m.name, "project")}
                            disabled={!connected || !m.loadable || Boolean(pendingEdit)}
                            className="rounded-lg border border-borderSubtle px-2 py-1 text-themeBlue hover:border-themeBlue disabled:opacity-40"
                          >
                            {pendingEdit?.source === "project" && pendingEdit.map === m.name
                              ? "加载中…" : "编辑"}
                          </button>
                        </div>
                      </div>
                    );
                  })}
                </div>
              </div>
            )}
            {groups.map((group) => (
              <div key={group.name}>
                <div className="mb-1.5 flex items-center justify-between">
                  <p className="font-[RobotoMono] text-xs uppercase tracking-wider text-themeTextGray">
                    {group.name}
                  </p>
                  <button
                    onClick={() => deleteGroup(group.name)}
                    className="text-[10px] text-themeTextGray hover:text-statusRed"
                  >
                    {t("delete group")}
                  </button>
                </div>
                {group.maps.length === 0 ? (
                  <p className="px-2 text-xs text-themeTextGray/60">
                    {t("No maps in this group.")}
                  </p>
                ) : (
                  <div className="space-y-1.5">
                    {group.maps.map((m) => {
                      const isActive = matchesActiveMap(active, {
                        group: group.name,
                        map: m.name,
                        source: "ui",
                      });
                      const isRenaming =
                        renaming?.group === group.name &&
                        renaming?.map === m.name;
                      return (
                        <div
                          key={m.name}
                          className={`flex flex-wrap items-center gap-3 rounded-lg border px-3 py-2 ${
                            isActive
                              ? "border-themeBlue/50 bg-themeBlue/5"
                              : "border-borderSubtle bg-bgSurface"
                          }`}
                        >
                          {isRenaming ? (
                            <input
                              autoFocus
                              className={`${inputClass} flex-1 py-1`}
                              value={renaming.value}
                              onChange={(e) =>
                                setRenaming((p) => ({
                                  ...p,
                                  value: e.target.value,
                                }))
                              }
                              onKeyDown={(e) => {
                                if (e.key === "Enter") commitRename();
                                if (e.key === "Escape") setRenaming(null);
                              }}
                            />
                          ) : (
                            <span className="min-w-0 flex-1 truncate text-sm font-semibold text-textWhiteHover">
                              {m.name}
                              {isActive && (
                                <span className="ml-2 rounded border border-themeBlue/40 px-1.5 py-0.5 text-[10px] text-themeBlue">
                                  {t("active")}
                                </span>
                              )}
                              <span className="ml-2 font-[RobotoMono] text-[11px] text-themeTextGray">
                                {m.routes.length} {t("route(s)")}
                              </span>
                            </span>
                          )}
                          <div className="flex shrink-0 items-center gap-1.5 font-[RobotoMono] text-xs">
                            {isRenaming ? (
                              <>
                                <button
                                  onClick={commitRename}
                                  className="rounded-lg border border-themeBlue px-2 py-1 text-themeBlue hover:bg-themeBlue hover:text-white"
                                >
                                  {t("Save")}
                                </button>
                                <button
                                  onClick={() => setRenaming(null)}
                                  className="rounded-lg border border-borderSubtle px-2 py-1 text-themeTextGray"
                                >
                                  {t("Cancel")}
                                </button>
                              </>
                            ) : (
                              <>
                                <button
                                  onClick={() => switchMap(group.name, m.name)}
                                  disabled={!connected || isActive}
                                  className="rounded-lg border border-themeBlue px-2 py-1 text-themeBlue hover:bg-themeBlue hover:text-white disabled:opacity-40"
                                >
                                  {t(isActive ? "Loaded" : "Switch")}
                                </button>
                                <button
                                  onClick={() => editMap(group.name, m.name)}
                                  disabled={!connected || Boolean(pendingEdit)}
                                  className="rounded-lg border border-borderSubtle px-2 py-1 text-themeBlue hover:border-themeBlue disabled:opacity-40"
                                >
                                  {pendingEdit?.group === group.name && pendingEdit?.map === m.name
                                    ? "加载中…" : "编辑"}
                                </button>
                                <button
                                  onClick={() =>
                                    setRenaming({
                                      group: group.name,
                                      map: m.name,
                                      value: m.name,
                                    })
                                  }
                                  className="rounded-lg border border-borderSubtle px-2 py-1 text-themeTextGray hover:border-themeBlue hover:text-themeBlue"
                                >
                                  {t("Rename")}
                                </button>
                                <button
                                  onClick={() => deleteMap(group.name, m.name)}
                                  aria-label={`Delete ${m.name}`}
                                  className="px-1 text-themeTextGray hover:text-statusRed"
                                >
                                  ×
                                </button>
                              </>
                            )}
                          </div>
                        </div>
                      );
                    })}
                  </div>
                )}
              </div>
            ))}
          </div>
        )}
      </DashboardCard>
    </div>
  );
};

export default MapsPage;
