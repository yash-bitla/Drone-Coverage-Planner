export function hms(seconds: number): string {
  const s = Math.max(0, Math.round(seconds));
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const sec = s % 60;
  return `${h}:${String(m).padStart(2, "0")}:${String(sec).padStart(2, "0")}`;
}

export const km = (meters: number): string => `${(meters / 1000).toFixed(1)} km`;
export const pct = (value: number): string => `${value.toFixed(1)}%`;
