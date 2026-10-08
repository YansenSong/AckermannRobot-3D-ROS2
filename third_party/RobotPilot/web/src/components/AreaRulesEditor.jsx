import React, { useCallback, useEffect, useRef, useState } from "react";
import { toast } from "react-toastify";

import Map from "./Map";
import { KeepoutZonesEditor } from "./KeepoutZones";
import useKeepoutZones from "../shared/hooks/useKeepoutZones";

const LABELS = {
  keepout: "禁行区",
  speed: "限速区",
  wall: "虚拟墙",
  closure: "临时封闭区",
};

const INITIAL_FORM = {
  type: "keepout", name: "", limit_mps: "0.3", minutes: "30", width: "0.1",
};

const inputClass =
  "min-h-9 rounded-lg border border-borderSubtle bg-bgCard px-2 text-xs text-textWhiteHover";

const AreaRulesEditor = () => {
  const rulesState = useKeepoutZones();
  const { ready, mapKey, upsertRule } = rulesState;
  const [form, setForm] = useState(INITIAL_FORM);
  const [drawing, setDrawing] = useState(false);
  const formRef = useRef(form);
  const upsertRef = useRef(upsertRule);
  const mapKeyRef = useRef(mapKey);
  formRef.current = form;
  upsertRef.current = upsertRule;

  const stopDrawing = useCallback(() => {
    setDrawing(false);
  }, []);

  useEffect(() => {
    if (drawing && !ready) stopDrawing();
  }, [drawing, ready, stopDrawing]);

  useEffect(() => {
    if (mapKeyRef.current && mapKeyRef.current !== mapKey) stopDrawing();
    mapKeyRef.current = mapKey;
  }, [mapKey, stopDrawing]);

  const handleAreaDraw = useCallback((geometry) => {
      if (!geometry) {
        toast.warn("请在地图上拖出足够大的区域或线段");
        return;
      }
      const current = formRef.current;
      const rule = {
        id: globalThis.crypto?.randomUUID?.() || `area-${Date.now()}-${Math.random()}`,
        name: current.name.trim(), type: current.type, ...geometry,
      };
      if (current.type === "speed") rule.limit_mps = Number(current.limit_mps);
      if (current.type === "closure") {
        rule.expires_at = new Date(Date.now() + Number(current.minutes) * 60000).toISOString();
      }
      if (upsertRef.current(rule)) toast.info("规则已发送，等待机器人确认");
  }, []);

  const startDrawing = () => {
    if (!ready) return;
    setDrawing(true);
  };

  const setField = (key) => (event) =>
    setForm((current) => ({ ...current, [key]: event.target.value }));
  const valid = form.name.length <= 100 &&
    (form.type !== "speed" || (Number.isFinite(Number(form.limit_mps)) &&
      Number(form.limit_mps) >= 0.01 && Number(form.limit_mps) <= 5)) &&
    (form.type !== "closure" || (Number.isFinite(Number(form.minutes)) &&
      Number(form.minutes) > 0 && Number(form.minutes) <= 525600)) &&
    (form.type !== "wall" || (Number.isFinite(Number(form.width)) &&
      Number(form.width) >= 0.01 && Number(form.width) <= 5));

  return (
    <div className="space-y-3">
      <p className="text-sm text-themeTextGray">
        禁行区、虚拟墙和临时封闭区约束路径规划并触发行驶保护；限速区约束实际速度。规则按当前地图保存。
      </p>
      <div className="flex flex-wrap items-center gap-2 font-[RobotoMono]">
        <select aria-label="地图规则类型" value={form.type} onChange={setField("type")}
          className={inputClass}>
          {Object.entries(LABELS).map(([value, label]) =>
            <option key={value} value={value}>{label}</option>)}
        </select>
        <input aria-label="规则名称" placeholder="名称（可选）" value={form.name}
          onChange={setField("name")} className={`${inputClass} w-36`} />
        {form.type === "speed" && <label className="text-xs text-themeTextGray">
          最高速度 m/s <input aria-label="最高速度" type="number" min="0.01" max="5"
            step="0.01" value={form.limit_mps} onChange={setField("limit_mps")}
            className={`${inputClass} ml-1 w-20`} />
        </label>}
        {form.type === "closure" && <label className="text-xs text-themeTextGray">
          封闭分钟 <input aria-label="封闭时长" type="number" min="1" max="525600"
            step="1" value={form.minutes} onChange={setField("minutes")}
            className={`${inputClass} ml-1 w-20`} />
        </label>}
        {form.type === "wall" && <label className="text-xs text-themeTextGray">
          墙宽 m <input aria-label="虚拟墙宽度" type="number" min="0.01" max="5"
            step="0.01" value={form.width} onChange={setField("width")}
            className={`${inputClass} ml-1 w-20`} />
        </label>}
        <button onClick={drawing ? stopDrawing : startDrawing}
          disabled={!drawing && (!ready || !valid)}
          className={`min-h-9 rounded-lg border px-3 text-xs font-semibold disabled:opacity-40 ${
            drawing ? "border-statusRed text-statusRed" : "border-themeBlue text-themeBlue"
          }`}>
          {drawing ? "结束绘制" : "在地图上绘制"}
        </button>
      </div>
      <p className="text-xs text-themeTextGray">
        {drawing ? "按住并拖动地图绘制；矩形用于区域规则，线段用于虚拟墙。" :
          ready ? `当前地图有 ${rulesState.rules.length} 条规则。` : "等待机器人地图规则服务。"}
      </p>
      {rulesState.error && <p role="alert" className="text-xs text-statusRed">{rulesState.error}</p>}
      <div className="h-[340px] min-w-0 sm:h-[440px] xl:h-[500px]">
        <Map drawingArea={drawing} areaDrawType={form.type}
          areaWallWidth={Number(form.width)} onAreaDraw={handleAreaDraw} />
      </div>
      <details className="rounded-lg border border-borderSubtle p-3">
        <summary className="cursor-pointer text-xs text-themeBlue">规则列表与坐标录入</summary>
        <div className="mt-3"><KeepoutZonesEditor {...rulesState} /></div>
      </details>
    </div>
  );
};

export default AreaRulesEditor;
