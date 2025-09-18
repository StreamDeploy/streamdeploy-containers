import fs from "fs-extra";
import path from "path";
import { glob } from "glob";

const root = process.cwd();
await fs.ensureDir(path.join(root, "dist"));

const owner = process.env.GH_OWNER || process.env.GITHUB_REPOSITORY_OWNER || "local";
const imagePrefix = process.env.IMAGE_PREFIX || "sd-";
const registry = process.env.REGISTRY || `ghcr.io/${owner}`;

const metas = await glob("containers/*/meta.json");
const legacy = [];
const v1 = [];
const introPath = path.join(root, "marketplaceIntro.md");
const intro = (await fs.pathExists(introPath)) ? await fs.readFile(introPath, "utf8") : "";
const sha = process.env.GITHUB_SHA || "local";

for (const file of metas) {
  const dir = path.dirname(file);
  const meta = await fs.readJson(file);
  const slug = meta.slug;
  const image = meta.ghcr_image || `${registry}/${imagePrefix}${slug}`;

  // Read sources from disk if present, fall back to meta definitions
  const dockerfilePath = path.join(dir, "Dockerfile");
  const entrypointPath = path.join(dir, "entrypoint.sh");
  const dockerfile = (await fs.pathExists(dockerfilePath)) ? await fs.readFile(dockerfilePath, "utf8") : (meta.dockerfile || "");
  const entrypoint = (await fs.pathExists(entrypointPath)) ? await fs.readFile(entrypointPath, "utf8") : (meta.entrypoint || "");

  legacy.push({
    ...meta,
    image,
    tags: ["latest"],
    repoPath: dir
  });

  v1.push({
    slug: meta.slug,
    name: meta.name,
    tagline: meta.tagline,
    primaryHw: meta.primaryHw,
    whatItDoes: meta.whatItDoes,
    whyItSavesTime: meta.whyItSavesTime,
    architectures: meta.architectures,
    tags: meta.tags,
    dockerfile,
    entrypoint,
    ghcr_image: image,
    deployment_ready: meta.deployment_ready ?? true,
    quick_deploy_enabled: meta.quick_deploy_enabled ?? true,
    fork_customize_enabled: meta.fork_customize_enabled ?? true,
    pi_optimized: meta.pi_optimized ?? false,
    default_env: meta.default_env ?? {},
    health_check_command: meta.health_check_command ?? "",
    resource_limits: meta.resource_limits ?? { cpu: "1.0", memory: "512Mi" }
  });
}

await fs.writeJson(path.join(root, "containers.json"), { marketplaceIntro: intro, containers: legacy }, { spaces: 2 });
await fs.writeJson(path.join(root, "dist", "marketplace.v1.json"), { version: 1, sha, marketplaceIntro: intro, containers: v1 }, { spaces: 2 });

console.log(`✅ Wrote containers.json and dist/marketplace.v1.json (${v1.length} items).`);