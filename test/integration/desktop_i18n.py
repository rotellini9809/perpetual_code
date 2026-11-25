"""UI language: English default, complete Italian catalog, and the Settings switch applied at restart."""
import ast
from pathlib import Path
from string import Formatter
from types import SimpleNamespace
from unittest.mock import patch
from PySide6.QtWidgets import QMessageBox
from perpetual_app.controller import Controller
from perpetual_app.window import MainWindow
from perpetual_app.i18n import set_language,language,load_language,SETTING
from perpetual_app.translations_it import ITALIAN

APP=Path(__file__).resolve().parents[2]/'perpetual_app'


def literal_keys():
    """Every t('literal') in the app; dynamic tables are listed in table_keys()."""
    keys=set()
    for path in APP.glob('*.py'):
        for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
            if (isinstance(node,ast.Call) and isinstance(node.func,ast.Name) and node.func.id=='t' and node.args
                    and isinstance(node.args[0],ast.Constant) and isinstance(node.args[0].value,str)):
                keys.add(node.args[0].value)
    return keys


def table_keys():
    """English texts kept in tables and translated with t(value) when shown."""
    from perpetual_app import airbrakes,imu_calibration_page,sensor_details,controller,dashboard_page
    keys={text for _,text in airbrakes.REASONS}
    keys|=set(controller.REASONS.values())|set(dashboard_page.CONFIG_ERRORS.values()) # Dashboard checklist.
    keys|=set(imu_calibration_page.STATES.values())|set(imu_calibration_page.CAUSES.values())|{'Not read'}
    keys|={title for _,title,*_ in sensor_details.SensorDetails.channels}
    keys|={'Control enabled','Control not active'} # Airbrake phase names from flight_data, drawn by flight_plot.
    return keys


def fields(text): return {name for _,name,_,_ in Formatter().parse(text) if name is not None}


def check_catalog():
    used=literal_keys()|table_keys()
    missing=sorted(used-ITALIAN.keys()-{''})
    assert not missing,'Missing Italian translations:\n'+'\n'.join(missing)
    unused=sorted(ITALIAN.keys()-used)
    assert not unused,'Unused catalog entries:\n'+'\n'.join(unused)
    wrong=[key for key,value in ITALIAN.items() if fields(key)!=fields(value)]
    assert not wrong,'Placeholders differ:\n'+'\n'.join(wrong)


def window_texts(app):
    controller=Controller(port_scanner=lambda:[]); controller.timer.stop()
    window=MainWindow(controller); window.show(); app.processEvents()
    texts=(window.nav[1].text(),window.dashboard_page.preflight_button.text(),window.settings_page.language.currentData())
    window._allow_close=True; window.close(); app.processEvents()
    return texts


def check_translations(app):
    check_catalog()
    set_language('en'); assert window_texts(app)==('Configuration','Recheck','en')
    set_language('it'); assert window_texts(app)==('Configurazione','Ricontrolla','it')
    for saved,expected in (('it','it'),('en','en'),('xx','en'),(None,'en')): # Unknown or unset falls back to English.
        load_language(SimpleNamespace(value=lambda key,default,saved=saved:saved if saved is not None else default)); assert language()==expected


def check_language_switch(app,wait):
    set_language('en')
    controller=Controller(port_scanner=lambda:[]); controller.timer.stop()
    window=MainWindow(controller); window.show(); app.processEvents()
    combo=window.settings_page.language; apply=window.settings_page.language_apply
    try:
        with patch.object(window.settings,'setValue') as save, patch('perpetual_app.window.QMessageBox.question',return_value=QMessageBox.No) as ask:
            assert not apply.isEnabled(); combo.setCurrentIndex(combo.findData('it'))
            assert apply.isEnabled() and not save.called # Choosing alone saves nothing.
            apply.click()
            save.assert_called_once_with(SETTING,'it'); assert ask.called and not window.restart_requested and window.isVisible()
            assert not apply.isEnabled() # Saved, waiting for a restart.
            ask.reset_mock(); combo.setCurrentIndex(combo.findData('en')); assert apply.isEnabled(); apply.click() # The running language needs no restart.
            assert not ask.called and save.call_args.args==(SETTING,'en') and not apply.isEnabled()
        with patch.object(window.settings,'setValue'), patch('perpetual_app.window.QMessageBox.question',return_value=QMessageBox.Yes):
            combo.setCurrentIndex(combo.findData('it')); apply.click()
            assert window.restart_requested; wait(lambda:not window.isVisible()) # Normal close; main.py then relaunches.
        from perpetual_app import main
        with patch('perpetual_app.main.QProcess.startDetached') as start, patch.object(main.sys,'argv',['perpetual_app/main.py','--port','COM9']):
            main.relaunch()
        program,arguments=start.call_args.args
        assert arguments[-2:]==['--port','COM9'] and arguments[0].endswith('main.py') # Same options in the new instance.
    finally:
        window._allow_close=True; window.close(); app.processEvents()
