import { hms } from "../format";

const SPEEDS = [60, 300, 600, 1800, 3600];

interface Props {
  t: number;
  end: number;
  playing: boolean;
  speed: number;
  onSeek: (t: number) => void;
  onPlayPause: () => void;
  onSpeed: (speed: number) => void;
}

export function PlaybackBar({ t, end, playing, speed, onSeek, onPlayPause, onSpeed }: Props) {
  return (
    <div className="playback">
      <button type="button" onClick={onPlayPause} disabled={end === 0}>
        {playing ? "Pause" : "Play"}
      </button>
      <input type="range" min={0} max={end} step={1} value={t} onChange={(e) => onSeek(Number(e.target.value))} />
      <span className="clock">
        {hms(t)} / {hms(end)}
      </span>
      <select value={speed} onChange={(e) => onSpeed(Number(e.target.value))}>
        {SPEEDS.map((s) => (
          <option key={s} value={s}>
            ×{s}
          </option>
        ))}
      </select>
    </div>
  );
}
