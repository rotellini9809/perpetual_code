"""Airbrake presentation shared by live status and recorded-flight inspection."""
from .configuration import condition_text,decode_condition
from .flight_data import phase_name
from .i18n import t

COLORS=('#187f73','#3c70b8','#b46a35','#9161b0')
REASONS=((16,'ABORT'),(8,'Invalid Kalman estimate'),(128,'Navigation data not updated'),
         (256,'Control inhibited by navigation'),(32,'Invalid controller command'),(64,'Physical output error'))


def channels(mask): return [i+1 for i in range(4) if mask & (1<<i)]


def control_description(flags,failed=0,conditions=(),rules=(),phase=None,bistage=False):
    """Separate ordinary gate waiting from safety closures; explain the known COAST 1 gate."""
    if flags & 2: return t('Active'),t('Conditions satisfied')
    velocity_check=bistage and phase==2 and bool(flags & 256) and not flags & 16
    causes=[t('Collecting the 8 velocity samples in COAST 1') if bit==256 and velocity_check else t(text) for bit,text in REASONS if flags & bit]
    if flags & 4:
        requested=[]
        for i,c in enumerate(conditions):
            if failed & (1<<i):
                text=condition_text(c,rules)
                requested.append(t('Required phase: {phase}',phase=text.removeprefix(t('Phase: '))) if c['type']=='phase' else t('Required condition: {condition}',condition=text))
        causes.extend(requested or [t('Rule conditions not satisfied')])
    state='ABORT' if flags & 16 else t('Safety closure') if flags & (8|32|64|128) else t('Bistage check') if velocity_check else t('Inhibited') if flags & 256 else t('Waiting') if flags & 4 else t('Not active')
    return state,'; '.join(causes)


def control_text(flags,failed=0,conditions=(),rules=()):
    if flags & 2: return t('Control enabled')
    state,detail=control_description(flags,failed,conditions,rules)
    return state+(' · '+detail if detail else '')


def recorded_conditions(configuration,bistage=False):
    conditions=[]
    for raw in configuration.get('airbrakes',{}).get('conditions',[]):
        c=decode_condition(raw)
        if c['type']=='phase': c['phase']=t(' or ').join(phase_name(i,bistage) for i in range(8) if raw['phase_mask'] & (1<<i)) or '—'
        conditions.append(c)
    return conditions
