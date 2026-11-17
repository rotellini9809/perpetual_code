"""Serial port metadata for USB-first discovery; opening a port is handled elsewhere."""
from dataclasses import dataclass
import re
import sys
from .i18n import t


@dataclass(frozen=True)
class Port:
    device: str
    transport: str = 'serial'
    identity: str = ''
    description: str = ''
    automatic: bool = False
    name: str = '' # Paired Bluetooth name, so several boards can be told apart.

    @property
    def label(self):
        kind={'usb':'USB','bluetooth':'Bluetooth','serial':t('Serial')}[self.transport]
        return ' · '.join(part for part in (kind,self.name,self.device) if part)


def bluetooth_name(address):
    """Read the paired device's cached name; never initiate pairing or discovery."""
    if sys.platform != 'win32': return ''
    import winreg
    try:
        path=rf'SYSTEM\CurrentControlSet\Services\BTHPORT\Parameters\Devices\{address.lower()}'
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,path) as key:
            value,_=winreg.QueryValueEx(key,'Name')
        return value.decode('utf-8',errors='replace').rstrip('\0') if isinstance(value,bytes) else str(value)
    except OSError:
        return ''


def describe_port(info, name_reader=bluetooth_name):
    device=info.device; hwid=(info.hwid or '').upper(); description=info.description or ''
    if 'BTHENUM' in hwid:
        # Windows incoming SPP ports use an all-zero remote address. They must
        # never be tried as an outgoing connection, regardless of localized names.
        match=re.search(r'&([0-9A-F]{12})_',hwid)
        address=match.group(1) if match else ''
        outgoing=bool(address and address!='000000000000' and '00001101-' in hwid)
        name=name_reader(address) if outgoing else ''
        recognized=not name or any(word in name.casefold() for word in ('perpetual','jdy-34','jdy34'))
        return Port(device,'bluetooth','bluetooth:'+address if outgoing else '',description,outgoing and recognized,name)
    if info.vid is not None:
        serial=info.serial_number or device
        # This firmware uses native RP2350 USB CDC, not an arbitrary USB/UART adapter.
        return Port(device,'usb',f'usb:{info.vid:04X}:{info.pid or 0:04X}:{serial}',description,
                    info.vid==0x2E8A and info.pid==0x000F)
    return Port(device,description=description)


def enumerate_ports():
    from serial.tools import list_ports
    return [describe_port(info) for info in sorted(list_ports.comports())]


def candidates(ports, preferred=None):
    preferred=preferred or {}
    result=[]; identities=set()
    for transport in ('usb','bluetooth'):
        group=[p for p in ports if p.transport==transport and
               (p.automatic or (p.identity and p.identity==preferred.get(transport)))]
        group.sort(key=lambda p:(p.identity!=preferred.get(transport),p.device))
        for port in group:
            if port.identity in identities: continue
            result.append(port); identities.add(port.identity)
    return result
