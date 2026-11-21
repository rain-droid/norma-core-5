/** The control ticket this page holds or waits with; sent with every request (see control.ts). */
let token: string | null = null;

export const controlToken = {
  get: (): string | null => token,
  set: (t: string | null): void => {
    token = t;
  },
};
