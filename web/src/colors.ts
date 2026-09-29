const PALETTE = ["#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c", "#0891b2", "#ca8a04", "#db2777"];

export const droneColor = (drone: number): string => PALETTE[drone % PALETTE.length];
