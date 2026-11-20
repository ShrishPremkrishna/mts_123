const BINDINGS = {
  blue: ['KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE'],
  red: ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Comma', 'Period'],
};

export const KEY_GUIDE = {
  blue: 'W/S forward and back · A/D strafe · Q/E turn',
  red: '↑/↓ forward and back · ←/→ strafe · comma/period turn',
};

export function slotForKey(code) {
  return Object.keys(BINDINGS).find(slot => BINDINGS[slot].includes(code));
}

export function readDrive(keys, slot) {
  if (!Object.hasOwn(BINDINGS, slot)) return {forward: 0, left: 0, turn: 0};
  const [up, down, leftKey, right, ccw, cw] = BINDINGS[slot];
  const axis = (positive, negative) => Number(keys.has(positive)) - Number(keys.has(negative));
  let forward = axis(up, down);
  let left = axis(leftKey, right);
  if (forward && left) {
    forward *= Math.SQRT1_2;
    left *= Math.SQRT1_2;
  }
  return {forward, left, turn: axis(ccw, cw)};
}
