import fs from "fs-extra";
import path from "path";
import { glob } from "glob";

const root = process.cwd();
const tmplDir = path.join(root, "dist", "templates");
await fs.ensureDir(tmplDir);

const owner = process.env.GH_OWNER || process.env.GITHUB_REPOSITORY_OWNER || "local";
const imagePrefix = process.env.IMAGE_PREFIX || "sd-";
const registry = process.env.REGISTRY || `ghcr.io/${owner}`;

const metas = await glob("containers/*/meta.json");
const index = [];

function suggestDevices(meta) {
  const t = (meta.tags || []).map((x) => x.toLowerCase());
  const out = [];
  if (t.some((x) => ["camera", "gstreamer", "rtsp", "realsense"].includes(x))) out.push("/dev/video0");
  if (t.includes("coral")) out.push("/dev/bus/usb");
  return out;
}

for (const file of metas) {
  const dir = path.dirname(file);
  const meta = await fs.readJson(file);
  const slug = meta.slug;
  const image = meta.ghcr_image || `${registry}/${imagePrefix}${slug}`;
  const devices = suggestDevices(meta);

  const template = {
    apiVersion: "streamdeploy/v1",
    kind: "ContainerRelease",
    metadata: {
      name: slug,
      annotations: {
        "io.streamdeploy.slug": slug,
        "io.streamdeploy.tags": (meta.tags || []).join(",")
      }
    },
    spec: {
      image,
      tag: "latest",
      env: meta.default_env || {},
      healthcheck: meta.health_check_command || "",
      resources: meta.resource_limits || { cpu: "1.0", memory: "512Mi" },
      // Hints for UI / device agent:
      suggestedDevices: devices
    }
  };

  const outfile = path.join(tmplDir, `${slug}.json`);
  await fs.writeJson(outfile, template, { spaces: 2 });
  index.push({ slug, path: `templates/${slug}.json` });
}

await fs.writeJson(path.join(tmplDir, "index.json"), { version: 1, items: index }, { spaces: 2 });
console.log(`✅ Built ${index.length} release template(s).`);
