import { useCallback, useEffect, useRef, useState } from "react";

export const clampT = (t: number, end: number) => Math.min(t, end);
export const startT = (t: number, end: number) => (t >= end ? 0 : t);

export function usePlayback(end: number) {
  const [t, setT] = useState(0);
  const [playing, setPlayingRaw] = useState(false);
  const [speed, setSpeed] = useState(600);
  const tRef = useRef(0);

  const seek = useCallback((value: number) => {
    tRef.current = value;
    setT(value);
  }, []);

  const setPlaying = useCallback(
    (value: boolean) => {
      if (value) seek(startT(tRef.current, end));
      setPlayingRaw(value);
    },
    [end, seek],
  );

  useEffect(() => {
    if (tRef.current > end) seek(clampT(tRef.current, end));
  }, [end, seek]);

  useEffect(() => {
    if (!playing) return;
    let frame = 0;
    let last = performance.now();
    const tick = (now: number) => {
      tRef.current = Math.min(end, tRef.current + ((now - last) / 1000) * speed);
      last = now;
      setT(tRef.current);
      if (tRef.current >= end) setPlayingRaw(false);
      else frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frame);
  }, [playing, speed, end]);

  const reset = useCallback(() => {
    seek(0);
    setPlayingRaw(false);
  }, [seek]);

  return { t: clampT(t, end), seek, playing, setPlaying, speed, setSpeed, reset };
}
