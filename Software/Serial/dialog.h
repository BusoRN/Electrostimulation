/****************************************************************************
**
** Copyright (C) 2012 Denis Shienkov <denis.shienkov@gmail.com>
** Contact: http://www.qt-project.org/legal
**
** This file is part of the QtSerialPort module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and Digia.  For licensing terms and
** conditions see http://qt.digia.com/licensing.  For further information
** use the contact form at http://qt.digia.com/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU Lesser General Public License version 2.1 requirements
** will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** In addition, as a special exception, Digia gives you certain additional
** rights.  These rights are described in the Digia Qt LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3.0 as published by the Free Software
** Foundation and appearing in the file LICENSE.GPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU General Public License version 3.0 requirements will be
** met: http://www.gnu.org/copyleft/gpl.html.
**
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef DIALOG_H
#define DIALOG_H
//#define __WINDOWS__

#include <QDialog>



class QLabel;
class QLineEdit;
class QSpinBox;
class QPushButton;
class QComboBox;


class Dialog : public QDialog
{
    Q_OBJECT

public:
    Dialog(QWidget *parent = 0);


private slots:
    void transaction();
   // void showResponse(const QString &s);
    //void processError(const QString &s);
   //void processTimeout(const QString &s);
    void openSerial(void);
    void serialReceived();
    void updatePorts();

private:
    void setControlsEnabled(bool enable);
    int convertVoltageData(QString data);
    void convertBuffer(int data);
    char* itoa(int value, char* result, int base) ;
    // void closing();

private:
    QStringList voltageValue;
    int intVoltageValue;
    int transactionCount;
    QLabel *serialPortLabel;
    QComboBox *serialPortComboBox;
    QLabel *widthLabel;
    QSpinBox *widthSpinBox;
    QLabel *requestLabel;
    QLineEdit *requestLineEdit;
    QLabel *trafficLabel;
    QLabel *statusLabel;
    QPushButton *runButton;
    QLabel *freqLabel;
    QSpinBox *freqSpinBox;
    QLabel *phaseLabel;
    QComboBox *phasePortComboBox;
    //QPushButton *openButton;
    //QPushButton *updateButton;

};

#endif // DIALOG_H
