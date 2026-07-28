import { copyFileSync, existsSync, mkdirSync } from "node:fs";
import { dirname, resolve } from "node:path";

const source = resolve(".openai", "hosting.json");
const destination = resolve("dist", ".openai", "hosting.json");

if (!existsSync(source)) {
  throw new Error(`Hosting manifest not found: ${source}`);
}

mkdirSync(dirname(destination), { recursive: true });
copyFileSync(source, destination);
console.log(`Copied hosting manifest to ${destination}`);
