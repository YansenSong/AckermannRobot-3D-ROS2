import React, { useCallback, useContext, useEffect, useState } from "react";
import { AuthContext } from "../app/App";
import { apiFetch } from "../shared/api/apiFetch";
import { DashboardCard, EmptyState } from "../shared/ui/Dashboard";
import { useRoleAccess } from "../shared/auth/roleAccess";

const LABELS = {
  occupancy_map_reviewed: "二维占据地图已复核",
  point_cloud_reviewed: "三维点云已复核",
  map_alignment_reviewed: "二维/三维坐标一致性已复核",
  localization_tested: "定位结果已验证",
  safety_zones_reviewed: "安全区域已复核",
};
const EMPTY = Object.fromEntries(Object.keys(LABELS).map((key) => [key, false]));

export default function MapQualityReviewPanel({ group, map }) {
  const { robotId } = useContext(AuthContext);
  const writeAccess = useRoleAccess("Engineer");
  const adminAccess = useRoleAccess("Admin");
  const [reviews, setReviews] = useState([]);
  const [checks, setChecks] = useState(EMPTY);
  const [issues, setIssues] = useState("");
  const [error, setError] = useState("");
  const base = `/api/v1/robots/${encodeURIComponent(robotId || "")}/maps/catalog/${encodeURIComponent(group || "")}/${encodeURIComponent(map || "")}/quality-reviews`;
  const load = useCallback(async () => {
    if (!robotId || !group || !map) return;
    try {
      const response = await apiFetch(base);
      setReviews((await response.json()).reviews || []);
      setError("");
    } catch (cause) { setError(cause.message || "地图质量记录不可用"); }
  }, [base, group, map, robotId]);
  useEffect(() => { load(); }, [load]);
  if (!group || !map) return null;
  const submit = async (event) => {
    event.preventDefault();
    try {
      await apiFetch(base, { method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ checks, issues }) });
      setChecks(EMPTY); setIssues(""); await load();
    } catch (cause) { setError(cause.message || "地图质量提交失败"); }
  };
  const decide = async (review, status) => {
    try {
      await apiFetch(`/api/v1/robots/${encodeURIComponent(robotId)}/maps/quality-reviews/${encodeURIComponent(review.review_id)}`, {
        method: "PATCH", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ status, review_note: review.issues || "" }),
      });
      await load();
    } catch (cause) { setError(cause.message || "地图审核失败"); }
  };
  return <DashboardCard className="space-y-4 p-4">
    <div><h2 className="font-semibold">地图版本质量复核</h2><p className="text-xs text-themeTextGray">{group} / {map} · 提交时绑定服务器计算的当前文件校验和</p></div>
    <form onSubmit={submit} className="space-y-3">
      <div className="grid gap-2 sm:grid-cols-2">{Object.entries(LABELS).map(([key, label]) => <label key={key} className="flex items-center gap-2 text-sm">
        <input type="checkbox" checked={checks[key]} onChange={(event) => setChecks((prev) => ({ ...prev, [key]: event.target.checked }))} />{label}
      </label>)}</div>
      <label className="block text-sm">缺陷与复核备注<textarea maxLength={4000} value={issues} onChange={(event) => setIssues(event.target.value)} className="mt-1 block min-h-20 w-full rounded border border-borderSubtle bg-bgCard p-2" /></label>
      <button disabled={!writeAccess.allowed} title={!writeAccess.allowed ? writeAccess.reason : undefined} className="rounded border border-themeBlue px-3 py-2 text-sm text-themeBlue disabled:opacity-50">提交待审核记录</button>
    </form>
    {error && <p role="alert" className="text-sm text-statusRed">{error}</p>}
    {reviews.length === 0 ? <EmptyState title="暂无质量复核记录" description="提交后需管理员审核；记录不会自动激活地图。" /> :
      <ul className="divide-y divide-borderSubtle">{reviews.map((review) => <li key={review.review_id} className="py-2 text-xs">
        <strong>{review.status}</strong> · {review.version_id} · 提交人 {review.submitted_by} · {review.created_at}
        <p className="break-all text-themeTextGray">{review.checksum}</p>
        {review.issues && <p>{review.issues}</p>}
        {review.status === "SUBMITTED" && adminAccess.allowed && <div className="mt-2 flex gap-2"><button type="button" onClick={() => decide(review, "APPROVED")} className="rounded border border-statusGreen px-2 py-1 text-statusGreen">批准</button><button type="button" onClick={() => decide(review, "REJECTED")} className="rounded border border-statusRed px-2 py-1 text-statusRed">退回</button></div>}
      </li>)}</ul>}
  </DashboardCard>;
}
