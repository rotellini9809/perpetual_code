"""UI language: English text in the code, Italian catalog. Chosen in Settings, applied at the next start.

Wrap user-facing text in t('English text') at display time, never at import time. Values go in
placeholders: t('{count} rules applied', count=n). Error messages stay in English and are not wrapped.
"""
from .translations_it import ITALIAN

LANGUAGES = {'en': 'English', 'it': 'Italiano'}
_CATALOGS = {'en': {}, 'it': ITALIAN}
_current = 'en'
SETTING = 'language' # QSettings key; the stored value is a LANGUAGES code.


def set_language(code):
    global _current
    _current = code if code in _CATALOGS else 'en'


def language(): return _current


def t(text, **values):
    text = _CATALOGS[_current].get(text, text)
    return text.format(**values) if values else text


def use_decimal_point():
    """Numbers use a decimal point in every language, independent of the Windows regional format."""
    from PySide6.QtCore import QLocale
    QLocale.setDefault(QLocale.c())


def load_language(settings):
    """Apply the language saved in QSettings (English when unset or unknown)."""
    set_language(str(settings.value(SETTING, 'en')))
