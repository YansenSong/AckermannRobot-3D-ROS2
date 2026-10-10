import React, { useCallback, useContext, useEffect, useState } from "react";
import { AuthContext } from "../app/App";
import { apiFetch } from "../shared/api/apiFetch";
import { DashboardCard, EmptyState, SectionHeader } from "../shared/ui/Dashboard";
import useRobotStatus from "../shared/hooks/useRobotStatus";
import { useRoleAccess } from "../shared/auth/roleAccess";

export default function AssetsPage() {
  const { robotId } = useContext(AuthContext);
  const status = useRobotStatus();
  const writeAccess = useRoleAccess("Engineer");
  const mapId = status.snapshot?.current_map?.map_id || "";
  const [assets, setAssets] = useState([]);
  const [waypoints, setWaypoints] = useState([]);
  const [name, setName] = useState("");
  const [kind, setKind] = useState("equipment");
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  const base = `/api/v1/robots/${encodeURIComponent(robotId || "")}`;
  const load = useCallback(async () => {
    if (!robotId) return;
    try {
      const [assetResponse, waypointResponse] = await Promise.all([
        apiFetch(`${base}/assets?map_id=${encodeURIComponent(mapId)}`),
        mapId ? apiFetch(`${base}/waypoints?map_id=${encodeURIComponent(mapId)}`) : Promise.resolve(null),
      ]);
      setAssets((await assetResponse.json()).assets || []);
      setWaypoints(waypointResponse ? (await waypointResponse.json()).waypoints || [] : []);
      setError("");
    } catch (cause) { setError(cause.message || "资产数据加载失败"); }
  }, [base, mapId, robotId]);
  useEffect(() => { load(); }, [load]);
  const create = async (event) => {
    event.preventDefault();
    if (!mapId) { setError("机器人当前地图未知，无法创建地图资产。"); return; }
    setBusy(true);
    try {
      await apiFetch(`${base}/assets`, { method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ map_id: mapId, name, metadata: { kind, active: true } }) });
      setName(""); await load();
    } catch (cause) { setError(cause.message || "资产创建失败"); }
    finally { setBusy(false); }
  };
  const link = async (assetId, waypointId) => {
    try {
      await apiFetch(`${base}/assets/${encodeURIComponent(assetId)}/waypoints/${encodeURIComponent(waypointId)}`, { method: "PUT" });
      await load();
    } catch (cause) { setError(cause.message || "关联失败"); }
  };
  const unlink = async (assetId, waypointId) => {
    try {
      await apiFetch(`${base}/assets/${encodeURIComponent(assetId)}/waypoints/${encodeURIComponent(waypointId)}`, { method: "DELETE" });
      await load();
    } catch (cause) { setError(cause.message || "解除关联失败"); }
  };
  const setActive = async (asset, active) => {
    try {
      await apiFetch(`${base}/assets/${encodeURIComponent(asset.asset_id)}`, { method: "PATCH",
        headers: { "Content-Type": "application/json", "If-Match": String(asset.revision) },
        body: JSON.stringify({ name: asset.name, metadata: asset.metadata, active }) });
      await load();
    } catch (cause) { setError(cause.message || "资产更新失败"); }
  };
  return <div className="sectionHeight space-y-5 py-4 sm:py-6">
    <SectionHeader eyebrow="Inventory" title="巡检资产" description="资产绑定当前地图版本；基准照片媒体存储尚未接入。" />
    <DashboardCard className="p-4"><p className="mb-3 text-xs text-themeTextGray">当前地图：{mapId || "未知"}{status.stale ? " · 状态过期" : ""}</p>
      <form onSubmit={create} className="flex flex-wrap items-end gap-3">
        <label className="flex flex-col gap-1 text-sm">资产名称<input required maxLength={100} value={name} onChange={(event) => setName(event.target.value)} className="rounded border border-borderSubtle px-3 py-2" /></label>
        <label className="flex flex-col gap-1 text-sm">类型<select value={kind} onChange={(event) => setKind(event.target.value)} className="rounded border border-borderSubtle px-3 py-2"><option value="equipment">设备</option><option value="fire_safety">消防设施</option><option value="utility">管线/仪表</option><option value="other">其他</option></select></label>
        <button disabled={busy || !mapId || !writeAccess.allowed} title={!writeAccess.allowed ? writeAccess.reason : undefined} className="rounded bg-themeBlue px-4 py-2 text-white disabled:opacity-50">创建资产</button>
      </form>
    </DashboardCard>
    {error && <p role="alert" className="text-statusRed">{error}</p>}
    <DashboardCard className="p-4"><h2 className="mb-3 font-semibold">资产清单</h2>
      {assets.length === 0 ? <EmptyState title="当前地图暂无资产" description="需要 Engineer 权限并连接到可信的当前地图。" /> :
        <ul className="divide-y divide-borderSubtle">{assets.map((asset) => <li key={asset.asset_id} className="flex flex-wrap items-center justify-between gap-3 py-3">
          <div><strong>{asset.name}</strong><p className="text-xs text-themeTextGray">{asset.metadata?.kind || "未分类"} · 地图版本 {asset.map_version_id || "未知"} · 修订 {asset.revision} · {asset.metadata?.active === false ? "停用" : "启用"}</p>
            <div className="mt-2 flex flex-wrap items-center gap-2">
              {(asset.waypoint_ids || []).map((waypointId) => {
                const waypoint = waypoints.find((item) => item.waypoint_id === waypointId);
                return <span key={waypointId} className="inline-flex items-center gap-1 rounded border border-borderSubtle px-2 py-1 text-xs">
                  {waypoint?.name || waypointId}
                  <button type="button" disabled={!writeAccess.allowed} aria-label={`解除 ${asset.name} 与 ${waypoint?.name || waypointId} 的关联`} onClick={() => unlink(asset.asset_id, waypointId)} className="text-statusRed disabled:opacity-50">×</button>
                </span>;
              })}
              <select aria-label={`关联 ${asset.name} 到巡检点`} value="" disabled={!writeAccess.allowed} onChange={(event) => event.target.value && link(asset.asset_id, event.target.value)} className="rounded border border-borderSubtle px-2 py-1 text-xs disabled:opacity-50"><option value="">关联巡检点…</option>{waypoints.filter((wp) => !(asset.waypoint_ids || []).includes(wp.waypoint_id)).map((wp) => <option key={wp.waypoint_id} value={wp.waypoint_id}>{wp.name}</option>)}</select>
            </div>
          </div>
          <button type="button" disabled={!writeAccess.allowed} title={!writeAccess.allowed ? writeAccess.reason : undefined} onClick={() => setActive(asset, asset.metadata?.active === false)} className="rounded border border-borderSubtle px-3 py-1 text-xs disabled:opacity-50">{asset.metadata?.active === false ? "启用" : "停用"}</button>
        </li>)}</ul>}
    </DashboardCard>
  </div>;
}
