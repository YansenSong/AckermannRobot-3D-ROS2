const API_BASE = window.location.port === "3000"
  ? `http://${window.location.hostname}:5050`
  : "";
const credentials = "include";

export async function apiFetch(path, options = {}) {
  const method = (options.method || "GET").toUpperCase();
  const headers = new Headers(options.headers || {});
  if (!["GET", "HEAD", "OPTIONS"].includes(method)) {
    const response = await fetch(`${API_BASE}/api/v1/auth/csrf`, {
      credentials,
      cache: "no-store",
    });
    if (!response.ok) throw new Error("Authentication session is unavailable.");
    const { csrf_token: csrfToken } = await response.json();
    if (csrfToken) headers.set("X-CSRF-Token", csrfToken);
  }
  return fetch(`${API_BASE}${path}`, {
    ...options,
    headers,
    credentials,
  });
}
