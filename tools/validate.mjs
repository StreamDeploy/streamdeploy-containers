import fs from "fs-extra";
import path from "path";
import { glob } from "glob";
import Ajv from "ajv";
import addFormats from "ajv-formats";

const root = process.cwd();
const schema = await fs.readJson(path.join(root, "schema", "container-meta.schema.json"));

const ajv = new Ajv({ allErrors: true, strict: false });
addFormats(ajv);
const validate = ajv.compile(schema);

const metas = await glob("containers/*/meta.json");
let errors = 0;

for (const file of metas) {
  const data = await fs.readJson(file);
  const ok = validate(data);
  if (!ok) {
    console.error(`❌ Schema errors in ${file}`);
    console.error(validate.errors);
    errors++;
  }
  const slug = path.basename(path.dirname(file));
  if (data.slug !== slug) {
    console.error(`❌ slug mismatch in ${file}: "${data.slug}" != folder "${slug}"`);
    errors++;
  }
  const dir = path.dirname(file);
  if (!fs.existsSync(path.join(dir, "Dockerfile"))) {
    console.warn(`⚠️  Missing Dockerfile in ${dir}`);
  }
  if (!fs.existsSync(path.join(dir, "entrypoint.sh"))) {
    console.warn(`⚠️  Missing entrypoint.sh in ${dir}`);
  }
}

if (errors) {
  console.error(`\nValidation failed with ${errors} error(s).`);
  process.exit(1);
}
console.log(`✅ ${metas.length} meta.json file(s) validated.`);