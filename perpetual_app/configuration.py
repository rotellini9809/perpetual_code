"""Local configuration model and acknowledged firmware draft operations; no Qt."""
from copy import deepcopy
from dataclasses import dataclass, field
import math

from device_client import validate_command
from .i18n import t

CONDITION_TYPES = ['phase', 'apogee', 'altitude', 'altitude_range', 'speed', 'speed_range',
                   'since_launch_ms', 'since_apogee_ms', 'since_rule_ms']
COMPARISONS = ['<', '>', '<=', '>=']


@dataclass
class Plan:
    servos: list = field(default_factory=lambda: [dict(calibrated=False, initial_us=0, final_us=0) for _ in range(4)])
    rules: list = field(default_factory=list)
    airbrakes: dict = field(default_factory=lambda: dict(channel_mask=0,conditions=[]))

    def clone(self):
        return deepcopy(self)

    def airbrake_channel(self, channel): return bool(self.airbrakes['channel_mask'] & (1<<(channel-1)))

    def servo_channels(self):
        return {i for i in range(1,5) if self.airbrake_channel(i) or self.assigned('servo',i)}

    def assigned(self, kind, channel):
        return next((r for r in self.rules for a in r['outputs'] if a['kind']==kind and a['channel']==channel), None)


def rule_label(rules, identity):
    for i, rule in enumerate(rules):
        if rule['id'] == identity:
            name = rule['name']
            return f'{name} ({i+1})' if sum(r['name']==name for r in rules)>1 else name
    return t('Rule not available')


def next_rule_id(plan, baseline=None):
    # Do not reuse removed IDs still present in saved rules or dangling references.
    rules = plan.rules + (baseline.rules if baseline else [])
    used = {r['id'] for r in rules}
    used.update(c['reference'] for r in rules for c in r['conditions'] if c['type']=='since_rule_ms')
    candidate = max(used, default=0)+1
    return candidate if candidate<=0xFFFFFFFF else next(i for i in range(1,len(used)+2) if i not in used)


def condition_text(c, rules=()):
    kind = c['type']
    if kind == 'apogee': return t('Apogee detected')
    if kind == 'phase':
        phase=str(c.get('phase','descent'))
        if phase.isdigit():
            names={1:t('BOOST 1 / normal'),2:t('COAST 1 / normal'),3:'DESCENT',5:'BOOST 2',6:'COAST 2'}
            phase=t(' or ').join(name for i,name in names.items() if int(phase)&(1<<i))
        return t('Phase: ')+phase
    if kind in ('altitude','speed'):
        return f'{t("Altitude") if kind=="altitude" else t("|Vertical speed|")} {c["comparison"]} {c["value"]:g} {"m" if kind=="altitude" else "m/s"}'
    if kind in ('altitude_range','speed_range'):
        return f'{t("Altitude") if kind=="altitude_range" else t("|Vertical speed|")}: {c["value"]:g} … {c["upper"]:g} {"m" if kind=="altitude_range" else "m/s"}'
    reference = {'since_launch_ms':t('launch'),'since_apogee_ms':t('apogee'),'since_rule_ms':t('rule «{name}»',name=rule_label(rules,c.get('reference')))}[kind]
    return t('Since {reference}: {elapsed} ms',reference=reference,elapsed=c['elapsed_ms'])


def decode_condition(reply):
    kind = CONDITION_TYPES[reply['kind']]
    value = dict(type=kind)
    if kind == 'phase': value['phase'] = str(reply['phase_mask'])
    elif kind in ('altitude','speed'): value.update(comparison=COMPARISONS[reply['comparison']],value=reply['value'])
    elif kind in ('altitude_range','speed_range'): value.update(value=reply['value'],upper=reply['upper'])
    elif kind != 'apogee':
        value['elapsed_ms'] = reply['elapsed_ms']
        if kind == 'since_rule_ms': value['reference'] = reply['reference']
    return value


def encode_condition(c):
    kind = c['type']
    if kind not in CONDITION_TYPES: raise ValueError('Unknown condition type')
    if kind == 'apogee': return kind
    if kind == 'phase': return f'phase {c["phase"]}'
    if kind in ('altitude','speed'):
        allowed = ('<','>') if kind=='altitude' else ('<=','>=')
        if c['comparison'] not in allowed: raise ValueError('Unsupported comparison')
        return f'{kind} {c["comparison"]} {number(c["value"])}'
    if kind in ('altitude_range','speed_range'): return f'{kind} {number(c["value"])} {number(c["upper"])}'
    if kind == 'since_rule_ms': return f'{kind} {integer(c["reference"],1)} {integer(c["elapsed_ms"])}'
    return f'{kind} {integer(c["elapsed_ms"])}'


def integer(value, minimum=0):
    if type(value) is not int or not minimum <= value <= 0xFFFFFFFF: raise ValueError('Integer out of range')
    return value


def number(value):
    if not isinstance(value,(int,float)) or not math.isfinite(value): raise ValueError('Invalid numeric value')
    return format(value,'.9g')


def quoted(name):
    if not name.strip() or len(name)>47: raise ValueError('Rule name required: at most 47 ASCII characters')
    return '"'+name.replace('\\','\\\\').replace('"','\\"')+'"'


def read_plan(client):
    applied = client.request('config show')
    draft = client.request('config draft')
    meta = draft if draft['available'] else applied
    if not meta['available']: return None, applied, 'applied'
    view = meta['view']
    def read(command):
        result = client.request(command)
        if result['view'] != view: raise RuntimeError('Configuration changed while reading: reload')
        if command.startswith('output show ') and 'delay_ms' not in result:
            raise RuntimeError('Firmware not updated: load MAIN to use channel delays.')
        return result
    plan = Plan()
    for i in range(4):
        servo = read(f'servo show {i+1}')
        plan.servos[i] = {key:servo[key] for key in ('calibrated','initial_us','final_us')}
    air = read('airbrake show')
    plan.airbrakes = dict(channel_mask=air['channel_mask'],conditions=[decode_condition(read(f'airbrake show {i}')) for i in range(air['conditions'])])
    for rule_id in meta['rule_ids']:
        r = read(f'rule show {rule_id}')
        if 'delay_ms' not in r:
            raise RuntimeError('Firmware not updated: load MAIN to use the rule delay.')
        plan.rules.append(dict(id=rule_id,name=r['name'],delay_ms=r['delay_ms'],
            conditions=[decode_condition(read(f'condition show {rule_id} {i}')) for i in range(r['conditions'])],
            outputs=[{key:a[key] for key in ('kind','channel','delay_ms')} for a in
                     (read(f'output show {rule_id} {i}') for i in range(r['outputs']))]))
    return plan, applied, view


def removal_order(plan):
    remaining = {r['id']:r for r in plan.rules}
    order = []
    while remaining:
        referenced = {c['reference'] for r in remaining.values() for c in r['conditions'] if c['type']=='since_rule_ms'}
        free = [i for i in remaining if i not in referenced]
        if not free: raise ValueError('Circular dependencies in the configuration read')
        for i in free: order.append(i); del remaining[i]
    return order


def plan_commands(base, local):
    commands = [f'airbrake remove {i}' for i in reversed(range(len(base.airbrakes['conditions'])))]
    commands += [f'rule remove {i}' for i in removal_order(base)]
    if len(local.rules)>8: raise ValueError('At most 8 rules')
    ids = [integer(r['id'],1) for r in local.rules]
    if len(set(ids)) != len(ids): raise ValueError('Rule IDs must be distinct')
    for i, servo in enumerate(local.servos):
        if servo != base.servos[i]:
            if not servo['calibrated']: raise ValueError('Removing servo endpoints is not available')
            commands.append(f'servo set {i+1} {integer(servo["initial_us"],1)} {integer(servo["final_us"],1)}')
    for r in local.rules:
        commands.extend([f'rule add {r["id"]} {quoted(r["name"])}', f'rule delay {r["id"]} {integer(r["delay_ms"])}'])
    assigned = {('servo',i):'Airbrakes' for i in range(1,5) if local.airbrake_channel(i)}
    for r in local.rules:
        if len(r['conditions'])>8 or len(r['outputs'])>8: raise ValueError('At most 8 conditions and 8 outputs per rule')
        commands += [f'condition add {r["id"]} {encode_condition(c)}' for c in r['conditions']]
        for a in r['outputs']:
            if a['kind'] not in ('servo','pyro') or not 1 <= a['channel'] <= 4: raise ValueError('Invalid channel')
            key = (a['kind'],a['channel'])
            if key in assigned:
                raise ValueError(f'{a["kind"].upper()} {a["channel"]} is already assigned to rule «{assigned[key]}». Each channel can belong to only one rule.')
            assigned[key] = r['name']
            commands.append(f'output assign {r["id"]} {a["kind"]} {a["channel"]} {integer(a.get("delay_ms",0))}')
    mask=integer(local.airbrakes['channel_mask'])
    if mask>15 or len(local.airbrakes['conditions'])>8: raise ValueError('Invalid airbrake rule')
    commands += [f'airbrake channels {mask}']
    commands += [f'airbrake add {encode_condition(c)}' for c in local.airbrakes['conditions']]
    for command in commands: validate_command(command)
    return commands


def stage_plan(client, base, local):
    # Compile all local input before creating or changing a device draft.
    commands = plan_commands(base or Plan(), local)
    control = client.request('control status')
    meta = client.request('config draft')
    if meta['available']:
        if control['owner'] != control['transport']: raise RuntimeError('Board draft held by another connection')
        client.request('config discard')
    current, _, view = read_plan(client)
    if view != 'applied' or current != base:
        raise RuntimeError('The board configuration has changed. Reload before applying the local draft.')
    client.request('config begin')
    frozen, _, view = read_plan(client)
    if view != 'draft' or frozen != (base or Plan()):
        client.request('config discard')
        raise RuntimeError('The configuration changed before the draft was reserved: reload')
    for command in commands: client.request(command)
    return client.request('config validate')
