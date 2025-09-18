// This script can generate container directories from a TypeScript seed.
// Provide your definitions in `seed/seed-containers.ts` and run `npm run seed`.
import fs from 'fs-extra';
import path from 'path';
import { fileURLToPath } from 'url';

// eslint-disable-next-line @typescript-eslint/ban-ts-comment
// @ts-ignore – import user-provided definitions
import { containers, marketplaceIntro } from '../seed/seed-containers.ts';

const __dirname = path.dirname(fileURLToPath(import.meta.url));

async function main() {
  const root = path.resolve(__dirname, '..');
  const containersDir = path.join(root, 'containers');
  await fs.ensureDir(containersDir);

  // write marketplace intro
  await fs.writeFile(path.join(root, 'marketplaceIntro.md'), marketplaceIntro);

  for (const c of containers) {
    const dir = path.join(containersDir, c.slug);
    await fs.ensureDir(dir);
    const { dockerfile, entrypoint, ...meta } = c as any;
    await fs.writeJson(path.join(dir, 'meta.json'), meta, { spaces: 2 });
    if (typeof c.dockerfile === 'string' && c.dockerfile.trim()) {
      await fs.writeFile(path.join(dir, 'Dockerfile'), c.dockerfile);
    }
    if (typeof c.entrypoint === 'string' && c.entrypoint.trim()) {
      const ep = path.join(dir, 'entrypoint.sh');
      await fs.writeFile(ep, c.entrypoint);
      await fs.chmod(ep, 0o755);
    }
    const readme = `# ${c.name}\n\n**Tagline:** ${c.tagline}\n\n**Primary hardware:** ${c.primaryHw}\n\n## What it does\n${c.whatItDoes}\n\n## Why it saves time\n${c.whyItSavesTime}\n\n## Architectures\n${c.architectures.join(', ')}\n\n## Tags\n${c.tags.join(', ')}\n\n### Runtime notes\n- Mount devices as needed (e.g. /dev/video*, USB accelerators, etc.)\n- Set environment variables documented in Dockerfile comments/entrypoint.\n`;
    await fs.writeFile(path.join(dir, 'README.md'), readme);
  }
  console.log(`Seeded ${containers.length} container(s).`);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});