"""Airbrake presentation shared by live status and recorded-flight inspection."""
from .configuration import condition_text,decode_condition
from .flight_data import phase_name

COLORS=('#187f73','#3c70b8','#b46a35','#9161b0')
REASONS=((16,'ABORT'),(8,'Stima Kalman non valida'),(128,'Dati di navigazione non aggiornati'),
         (256,'Controllo inibito dalla navigazione'),(32,'Comando del controllore non valido'),(64,'Errore uscita fisica'))


def channels(mask): return [i+1 for i in range(4) if mask & (1<<i)]


def control_description(flags,failed=0,conditions=(),rules=(),phase=None,bistage=False):
    """Separate ordinary gate waiting from safety closures; explain the known COAST 1 gate."""
    if flags & 2: return 'Attivo','Condizioni soddisfatte'
    velocity_check=bistage and phase==2 and bool(flags & 256) and not flags & 16
    causes=['Raccolta degli 8 campioni di velocità in COAST 1' if bit==256 and velocity_check else text for bit,text in REASONS if flags & bit]
    if flags & 4:
        requested=[]
        for i,c in enumerate(conditions):
            if failed & (1<<i):
                text=condition_text(c,rules)
                requested.append('Fase richiesta: '+text.removeprefix('Fase: ') if c['type']=='phase' else 'Condizione richiesta: '+text)
        causes.extend(requested or ['Condizioni della regola non soddisfatte'])
    state='ABORT' if flags & 16 else 'Chiusura di sicurezza' if flags & (8|32|64|128) else 'Verifica bistadio' if velocity_check else 'Inibito' if flags & 256 else 'In attesa' if flags & 4 else 'Non attivo'
    return state,'; '.join(causes)


def control_text(flags,failed=0,conditions=(),rules=()):
    if flags & 2: return 'Controllo abilitato'
    state,detail=control_description(flags,failed,conditions,rules)
    return state+(' · '+detail if detail else '')


def recorded_conditions(configuration,bistage=False):
    conditions=[]
    for raw in configuration.get('airbrakes',{}).get('conditions',[]):
        c=decode_condition(raw)
        if c['type']=='phase': c['phase']=' o '.join(phase_name(i,bistage) for i in range(8) if raw['phase_mask'] & (1<<i)) or '—'
        conditions.append(c)
    return conditions
