import fs from 'fs';
import { AuthStorage, ModelRegistry } from '@mariozechner/pi-coding-agent';

const cfgRaw = fs.readFileSync('/home/node/.openclaw/openclaw.json', 'utf8');
const cfg = JSON.parse(cfgRaw);

const auth = AuthStorage.create('/home/node/.openclaw/agents/main/agent/auth.json');
const reg = new ModelRegistry(auth, '/home/node/.openclaw/agents/main/agent/models.json');
const catalog = reg.getAll();

function modelKey(provider, model) { return provider + '/' + model; }
function normalizeProviderId(provider) { return provider.trim().toLowerCase(); }

const rawAllowlist = Object.keys(cfg.agents?.defaults?.models ?? {});
const catalogKeys = new Set(catalog.map(entry => modelKey(entry.provider, entry.id)));
const configuredProviders = cfg.models?.providers ?? {};
const defaultProvider = 'anthropic';
const allowedKeys = new Set();

console.log('Raw allowlist:', rawAllowlist);
console.log('Has openai/gpt-5.2-pro in catalog:', catalogKeys.has('openai/gpt-5.2-pro'));
console.log('Configured providers:', Object.keys(configuredProviders));

for (const raw of rawAllowlist) {
  const trimmed = raw.trim();
  const slash = trimmed.indexOf('/');
  const parsed = slash === -1 
    ? { provider: defaultProvider, model: trimmed } 
    : { provider: trimmed.slice(0, slash).trim(), model: trimmed.slice(slash + 1).trim() };
  const key = modelKey(parsed.provider, parsed.model);
  const providerKey = normalizeProviderId(parsed.provider);
  const inCatalog = catalogKeys.has(key);
  const hasProvider = configuredProviders[providerKey] != undefined;
  console.log(raw, '->', key, '| inCatalog:', inCatalog, '| hasProvider:', hasProvider);
  if (inCatalog || hasProvider) { allowedKeys.add(key); }
}
console.log('Final allowedKeys:', Array.from(allowedKeys));
