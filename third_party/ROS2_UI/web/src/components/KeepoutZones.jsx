import React, { useState } from "react";

import useKeepoutZones from "../shared/hooks/useKeepoutZones";

const EMPTY = { type: "keepout", name: "", cx: "", cy: "", w: "", h: "",
  x1: "", y1: "", x2: "", y2: "", width: "0.1", limit_mps: "0.3", minutes: "30" };
const LABELS = { keepout: "禁行区", speed: "限速区", wall: "虚拟墙", closure: "临时封闭区" };
const inputClass = "w-full rounded-lg border border-borderSubtle bg-bgCard px-2 py-1 text-xs text-textWhiteHover placeholder:text-themeTextGray";

const readLegacy = () => {
  try {
    const saved = JSON.parse(localStorage.getItem("robotpilotKeepoutZones") || "[]");
    return Array.isArray(saved) ? saved : [];
  } catch {
    return [];
  }
};

export const KeepoutZonesEditor = ({ rules, ready, mapKey, error, upsertRule, deleteRule }) => {
  const [form, setForm] = useState(EMPTY);
  const [legacy] = useState(readLegacy);
  const set = (field) => (event) => setForm((old) => ({ ...old, [field]: event.target.value }));
  const field = (key, placeholder) => (
    <input key={key} value={form[key]} onChange={set(key)} placeholder={placeholder}
      inputMode="decimal" className={inputClass} aria-label={placeholder} />
  );

  const numbers = form.type === "wall"
    ? ["x1", "y1", "x2", "y2", "width"] : ["cx", "cy", "w", "h"];
  const valid = numbers.every((key) => form[key] !== "" && Number.isFinite(Number(form[key])))
    && (form.type === "wall" ? Number(form.width) > 0
      : Number(form.w) > 0 && Number(form.h) > 0)
    && (form.type !== "speed" || (Number.isFinite(Number(form.limit_mps)) && Number(form.limit_mps) > 0))
    && (form.type !== "closure" || (Number.isFinite(Number(form.minutes)) && Number(form.minutes) > 0));

  const add = () => {
    if (!valid) return;
    const rule = { id: globalThis.crypto?.randomUUID?.() || `area-${Date.now()}-${Math.random()}`,
      type: form.type, name: form.name.trim() };
    numbers.forEach((key) => { rule[key] = Number(form[key]); });
    if (form.type === "speed") rule.limit_mps = Number(form.limit_mps);
    if (form.type === "closure") {
      rule.expires_at = new Date(Date.now() + Number(form.minutes) * 60000).toISOString();
    }
    upsertRule(rule);
  };

  const legacyMissing = legacy.filter((old) => !rules.some((rule) => rule.id === `legacy-${old.id}`));

  return (
    <div className="flex w-full flex-col gap-3 font-[RobotoMono] text-xs">
      <p className={ready ? "text-themeBlue" : "text-statusYellow"}>
        {ready ? `机器人规则在线 · 地图 ${mapKey?.slice(0, 8)} · ${rules.length} 条`
          : "等待机器人地图规则服务，当前不可编辑"}
      </p>
      {error && <p role="alert" className="text-statusRed">{error}</p>}
      {rules.length > 0 && <div className="flex flex-col gap-1">
        {rules.map((rule) => <div key={rule.id}
          className="flex items-center justify-between gap-2 rounded-lg border border-borderSubtle px-2 py-1">
          <span className="truncate text-textWhiteHover">{LABELS[rule.type]} · {rule.name || rule.id}</span>
          <span className="text-themeTextGray">
            {rule.type === "speed" ? `≤ ${rule.limit_mps} m/s` : rule.type === "wall"
              ? `${rule.width} m 宽` : `${rule.w}×${rule.h} m`}
          </span>
          <button onClick={() => deleteRule(rule.id)} disabled={!ready}
            aria-label={`删除 ${rule.name || rule.id}`} className="text-statusRed disabled:opacity-40">删除</button>
        </div>)}
      </div>}
      <div className="grid grid-cols-2 gap-1.5 sm:grid-cols-5">
        <select value={form.type} onChange={set("type")} className={inputClass} aria-label="规则类型">
          {Object.entries(LABELS).map(([value, label]) => <option key={value} value={value}>{label}</option>)}
        </select>
        {field("name", "名称")}
        {form.type === "wall" ? <>
          {field("x1", "起点 X (m)")}{field("y1", "起点 Y (m)")}
          {field("x2", "终点 X (m)")}{field("y2", "终点 Y (m)")}
          {field("width", "墙宽 (m)")}
        </> : <>
          {field("cx", "中心 X (m)")}{field("cy", "中心 Y (m)")}
          {field("w", "宽 (m)")}{field("h", "高 (m)")}
        </>}
        {form.type === "speed" && field("limit_mps", "最高速度 (m/s)")}
        {form.type === "closure" && field("minutes", "封闭时长 (分钟)")}
      </div>
      <button onClick={add} disabled={!ready || !valid}
        className="self-start rounded-lg border border-themeBlue px-3 py-1 text-themeBlue hover:bg-themeBlue hover:text-white disabled:opacity-40">
        添加规则
      </button>
      {ready && legacyMissing.length > 0 && <div className="rounded-lg border border-statusYellow/40 p-2">
        <p className="mb-1 text-statusYellow">此浏览器有 {legacyMissing.length} 条旧禁行区，需要逐条导入机器人。</p>
        {legacyMissing.map((old) => <button key={old.id} className="mr-2 text-themeBlue underline"
          onClick={() => upsertRule({ id: `legacy-${old.id}`, name: old.name || "旧禁行区",
            type: "keepout", cx: Number(old.cx), cy: Number(old.cy),
            w: Number(old.w), h: Number(old.h) })}>导入 {old.name || old.id}</button>)}
      </div>}
    </div>
  );
};

const KeepoutZones = () => <KeepoutZonesEditor {...useKeepoutZones()} />;

export default KeepoutZones;
