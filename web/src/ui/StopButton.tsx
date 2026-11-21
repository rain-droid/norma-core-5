import { useEffect, useState } from 'react';
import { stopEverything, stopStore } from '../actions.ts';
import { useStore } from '../state/store.ts';

const OK_NOTE_MS = 2600;
const FAIL_NOTE_MS = 9000;

/** Always on screen, never disabled. Escape does the same thing. */
export function StopButton() {
  const presses = useStore(stopStore, (s) => s.presses);
  const note = useStore(stopStore, (s) => s.note);
  const [pressed, setPressed] = useState(false);
  const [shown, setShown] = useState(false);

  // Flash the cap when Escape is used, so the key press is visible too.
  useEffect(() => {
    if (presses === 0) return;
    setPressed(true);
    const id = window.setTimeout(() => setPressed(false), 180);
    return () => window.clearTimeout(id);
  }, [presses]);

  useEffect(() => {
    if (!note) return;
    setShown(true);
    const id = window.setTimeout(() => setShown(false), note.ok ? OK_NOTE_MS : FAIL_NOTE_MS);
    return () => window.clearTimeout(id);
  }, [note]);

  return (
    <div className="stop-wrap">
      {shown && note && (
        <div className="stop-note" data-ok={note.ok} role="alert">
          {note.text}
        </div>
      )}
      <button
        type="button"
        className="stop"
        data-pressed={pressed}
        onClick={() => void stopEverything()}
        aria-label="Stop the robot now"
        aria-keyshortcuts="Escape"
      >
        <span className="stop__cap">
          <span className="stop__word">STOP</span>
          <span className="stop__key">esc</span>
        </span>
      </button>
    </div>
  );
}
