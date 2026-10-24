"""Reusable Qt widgets and layout helpers."""
from pathlib import Path
from PySide6.QtCore import QDir
from PySide6.QtWidgets import QWidget, QLabel, QFrame, QVBoxLayout, QHBoxLayout, QPushButton, QTableWidget, QTableWidgetItem, QHeaderView



ASSETS=Path(__file__).with_name('assets')
QDir.addSearchPath('perpetual',str(ASSETS))


def fill_table(widget,rows):
    selected=widget.currentRow(); widget.setRowCount(len(rows))
    for i,values in enumerate(rows):
        for j,text in enumerate(values): widget.setItem(i,j,QTableWidgetItem(str(text)))
    if 0<=selected<len(rows): widget.selectRow(selected)


def label(text='', name=None):
    widget=QLabel(text)
    if name: widget.setObjectName(name)
    return widget


def button(text, callback=None, kind=None):
    widget=QPushButton(text)
    if kind: widget.setObjectName(kind)
    if callback: widget.clicked.connect(lambda:callback())
    return widget


def card(title=None):
    frame=QFrame(); frame.setObjectName('card')
    layout=QVBoxLayout(frame); layout.setContentsMargins(22,20,22,20); layout.setSpacing(14)
    if title: layout.addWidget(label(title,'cardTitle'))
    return frame,layout


def table(headers):
    widget=QTableWidget(0,len(headers)); widget.setHorizontalHeaderLabels(headers)
    widget.verticalHeader().hide(); widget.setShowGrid(False); widget.setAlternatingRowColors(False)
    widget.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
    widget.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
    widget.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
    widget.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    widget.verticalHeader().setDefaultSectionSize(43)
    return widget


def row(*widgets):
    layout=QHBoxLayout(); layout.setSpacing(12)
    for widget in widgets:
        if widget is None: layout.addStretch()
        else: layout.addWidget(widget)
    return layout


class Metric(QFrame):
    def __init__(self,title,detail=''):
        super().__init__(); self.setObjectName('card')
        layout=QVBoxLayout(self); layout.setContentsMargins(20,17,20,17)
        layout.addWidget(label(title.upper(),'eyebrow'))
        self.value=label('—','metric'); layout.addWidget(self.value)
        self.detail=label(detail,'hint'); self.detail.setWordWrap(True); layout.addWidget(self.detail)

    def set(self,value,detail=None):
        self.value.setText(str(value))
        if detail is not None: self.detail.setText(detail)
