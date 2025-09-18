import fs from "fs-extra";
import path from "path";
import { glob } from "glob";

const onlyChanged = process.env.ONLY_CHANGED === "true";
const changedArg = process.env.CHANGED_FILES || "";

function uniq(arr) {
  return [...new Set(arr)];
}

async function listAll() {
  const metas = await glob("containers/*/meta.json");
  return metas.map((f) => path.basename(path.dirname(f)));
}

async function listChanged() {
  const files = changedArg
    .split("\n")
    .map((s) => s.trim())
    .filter(Boolean);
  const slugs = [];
  for (const f of files) {
    if (!f.startsWith("containers/")) continue;
    const parts = f.split("/");
    if (parts.length >= 2) slugs.push(parts[1]);
  }
  return uniq(slugs);
}

const slugs = onlyChanged ? await listChanged() : await listAll();
process.stdout.write(JSON.stringify(slugs));