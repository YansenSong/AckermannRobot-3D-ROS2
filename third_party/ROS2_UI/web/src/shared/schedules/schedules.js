// 持久化任务日程通过模块级 store 在 Scheduler 页面与后台 runner 之间共享，
// 与 shared/events/eventLog.js 使用相同模式。
// 日程按本地时间在浏览器端触发导航操作，只有浏览器标签页打开时才会运行。
// 若要在机器人侧独立运行，还需要后端定时器和机器人侧执行器。

const STORAGE_KEY = "robotpilotSchedules";

let schedules = load();
const listeners = new Set();

function load() {
  try {
    return JSON.parse(localStorage.getItem(STORAGE_KEY) || "[]");
  } catch {
    return [];
  }
}

function persist() {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(schedules));
  } catch {
    // ignore quota errors
  }
}

function emit() {
  listeners.forEach((fn) => fn(schedules));
}

export function getSchedules() {
  return schedules;
}

export function subscribeSchedules(fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}

export function addSchedule(entry) {
  schedules = [
    ...schedules,
    {
      id: `${Date.now()}`,
      enabled: true,
      repeat: "daily", // 'daily' | 'once'
      lastRunKey: null,
      ...entry,
    },
  ];
  persist();
  emit();
}

export function updateSchedule(id, fields) {
  schedules = schedules.map((s) => (s.id === id ? { ...s, ...fields } : s));
  persist();
  emit();
}

export function removeSchedule(id) {
  schedules = schedules.filter((s) => s.id !== id);
  persist();
  emit();
}
