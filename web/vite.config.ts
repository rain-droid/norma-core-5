import { createReadStream, existsSync, statSync } from 'node:fs';
import type { IncomingMessage, ServerResponse } from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import react from '@vitejs/plugin-react';
import { defineConfig, type Connect, type Plugin } from 'vite';

/** Where the robot daemon listens. Override with ELROBOT_DAEMON=http://host:port. */
const DAEMON = process.env.ELROBOT_DAEMON ?? 'http://127.0.0.1:8765';
const HERE = path.dirname(fileURLToPath(import.meta.url));
/** The robot model the daemon serves at /assets/elrobot/ (repo/assets/elrobot). */
const LOCAL_ROBOT_DIR = path.resolve(HERE, '../assets/elrobot');
const ROBOT_PREFIX = '/assets/elrobot/';

const MIME: Record<string, string> = {
  '.urdf': 'application/xml; charset=utf-8',
  '.stl': 'model/stl',
  '.json': 'application/json',
};

/**
 * Serves /assets/elrobot/* from the daemon when it is up, and from the local
 * repo folder when it is not (or when the daemon answers with an error), so
 * the 3D twin and `?mock=1` work without the Python side running.
 */
function robotAssetsFallback(): Plugin {
  let daemonUp = false;
  let checkedAt = 0;

  async function daemonHas(url: string): Promise<Response | null> {
    const now = Date.now();
    // Don't hammer a dead port: re-probe at most once a second.
    if (!daemonUp && now - checkedAt < 1000) return null;
    checkedAt = now;
    try {
      const res = await fetch(DAEMON + url, { signal: AbortSignal.timeout(1500) });
      daemonUp = true;
      return res.ok ? res : null;
    } catch {
      daemonUp = false;
      return null;
    }
  }

  const middleware: Connect.NextHandleFunction = (
    req: IncomingMessage,
    res: ServerResponse,
    next: Connect.NextFunction,
  ) => {
    const url = req.url ?? '';
    if (req.method !== 'GET' || !url.startsWith(ROBOT_PREFIX)) return next();

    void (async () => {
      const upstream = await daemonHas(url);
      if (upstream?.body) {
        res.statusCode = 200;
        const type = upstream.headers.get('content-type');
        if (type) res.setHeader('Content-Type', type);
        res.setHeader('X-Robot-Assets', 'daemon');
        res.end(Buffer.from(await upstream.arrayBuffer()));
        return;
      }

      const rel = decodeURIComponent(url.slice(ROBOT_PREFIX.length).split('?')[0]);
      const file = path.resolve(LOCAL_ROBOT_DIR, rel);
      const inside = file.startsWith(LOCAL_ROBOT_DIR + path.sep);
      if (!inside || !existsSync(file) || !statSync(file).isFile()) {
        res.statusCode = 404;
        res.end('robot asset not found');
        return;
      }
      res.statusCode = 200;
      res.setHeader('Content-Type', MIME[path.extname(file)] ?? 'application/octet-stream');
      res.setHeader('Content-Length', statSync(file).size);
      res.setHeader('Cache-Control', 'max-age=3600');
      res.setHeader('X-Robot-Assets', 'local-fallback');
      createReadStream(file).pipe(res);
    })().catch(next);
  };

  return {
    name: 'elrobot-assets-fallback',
    configureServer(server) {
      server.middlewares.use(middleware);
    },
    configurePreviewServer(server) {
      server.middlewares.use(middleware);
    },
  };
}

const proxy = {
  '/api': { target: DAEMON, changeOrigin: true },
  '/assets': { target: DAEMON, changeOrigin: true },
  '/ws': { target: DAEMON, ws: true, changeOrigin: true },
};

export default defineConfig({
  plugins: [react(), robotAssetsFallback()],
  build: {
    outDir: 'dist',
    // The daemon owns /assets (robot model), so the bundle goes under /static.
    assetsDir: 'static',
    chunkSizeWarningLimit: 1200,
  },
  server: { host: '127.0.0.1', port: 5173, proxy },
  preview: { host: '127.0.0.1', port: 4173, proxy },
});
