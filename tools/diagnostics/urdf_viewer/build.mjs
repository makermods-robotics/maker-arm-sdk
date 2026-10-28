import { cp, mkdir } from 'node:fs/promises';
await mkdir(new URL('./static/vendor/', import.meta.url), { recursive: true });
for (const pkg of ['three', 'urdf-loader']) {
  await cp(new URL(`./node_modules/${pkg}/`, import.meta.url), new URL(`./static/vendor/${pkg}/`, import.meta.url), { recursive: true });
}
console.log('Viewer assets built locally; no CDN required.');
