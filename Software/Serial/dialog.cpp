/****************************************************************************
**
**
**
****************************************************************************/

#include "dialog.h"

#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QSpinBox>
#include <QPushButton>
#include <QGridLayout>
#include <QDebug>
#include <QList>
#include <QString>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTimer>

//Rememder to define MAC on andrea's laptop
#if defined(__WIN32__) || defined(_WIN32) || defined(WIN32) || defined(__WINDOWS__) || defined(__TOS_WIN__)

  #include <windows.h>

  inline void delay( unsigned long ms )
    {
    Sleep( ms );
    }

#else  /* presume POSIX */

  #include <unistd.h>

  inline void delay( unsigned long ms )
    {
    usleep( ms * 1000 );
    }

#endif
QT_USE_NAMESPACE

int start = 0;
QSerialPort *serial; //constructor of my pserial port
QTimer *timer;
QGridLayout *mainLayout = new QGridLayout;
QSerialPortInfo QSPInfo[8];
bool status = false;
QByteArray buso;
char buffer[4];

char widthChar[3];

Dialog::Dialog(QWidget *parent)
    : QDialog(parent)
    , transactionCount(0)
    , serialPortLabel(new QLabel(tr("Serial port:")))
    , serialPortComboBox(new QComboBox())
    , widthLabel(new QLabel(tr("Select the width for pulse [ms]:")))
    , widthSpinBox(new QSpinBox())
    //, requestLabel(new QLabel(tr("Request:")))
    //, requestLineEdit(new QLineEdit(tr("Who are you?")))
    , trafficLabel(new QLabel(tr("No traffic.")))
    , statusLabel(new QLabel(tr("Status: Not running.")))
    , runButton(new QPushButton(tr("Start")))
    , freqLabel(new QLabel(tr("Select the frequency:")))
    ,freqSpinBox(new QSpinBox())
    ,phaseLabel(new QLabel(tr("Select the wayform type:")))
    ,phasePortComboBox(new QComboBox())
    //, openButton(new QPushButton(tr("Open")))
    //, updateButton(new QPushButton(tr("Update Ports")))

{
    int i= 0;

        foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
        {
            int x = QString::compare(info.manufacturer(), "Texas Instruments", Qt::CaseInsensitive);
            int y = QString::compare(info.manufacturer(), "TI", Qt::CaseInsensitive);
            if((x&y) == 0)
            {
                serialPortComboBox->addItem(info.description()+" - "+info.portName());
                QSPInfo[i] = info;
                i++;
           }
        }

    QStringList myOptions;
    myOptions << "Monophasic wave"<<"Biphasic wave";
    phasePortComboBox->addItems(myOptions);

    widthSpinBox->setRange(1, 500);
    widthSpinBox->setValue(1);
    freqSpinBox->setValue(1);
    freqSpinBox->setRange(1,400);


    mainLayout->addWidget(serialPortLabel, 0, 0);
    mainLayout->addWidget(serialPortComboBox, 0, 1);
    mainLayout->addWidget(widthLabel, 1,0);
    mainLayout->addWidget(widthSpinBox, 1,1 );
    mainLayout->addWidget(freqLabel, 2,0);
    mainLayout->addWidget(freqSpinBox, 2,1 );
    mainLayout->addWidget(phaseLabel, 3,0);
    mainLayout->addWidget(phasePortComboBox, 3,1 );
    mainLayout->addWidget(runButton, 5, 1);
   // mainLayout->addWidget(openButton, 5,0);
   // mainLayout->addWidget(updateButton, 4,0);
    mainLayout->addWidget(trafficLabel, 4, 0, 1, 4);
    mainLayout->addWidget(statusLabel, 4, 1, 1, 5);
    setLayout(mainLayout);

    timer = new QTimer(this);
    serial = new QSerialPort(this);

    setWindowTitle(tr("Blocking Master"));
    serialPortComboBox->setFocus();


    connect(runButton, SIGNAL(clicked()),
            this, SLOT(transaction()));

   //connect(openButton, SIGNAL(clicked()),
    //        this, SLOT(openSerial()));

   //connect(updateButton, SIGNAL(clicked()),
     //       this, SLOT(updatePorts()));
    connect(timer, SIGNAL(timeout()), this,
            SLOT(updatePorts()));
    timer->start(1000);
}

void Dialog::transaction()
{
    bool status = false;

    int index = serialPortComboBox->currentIndex();
    trafficLabel->setText(QSPInfo[index].portName());

    serial->setPortName(QSPInfo[index].portName());//

    status = serial->open(QIODevice::ReadWrite);

    if(status)
    {
        serial->setBaudRate(QSerialPort::Baud9600); // to set the wished Baudrate
        serial->setDataBits(QSerialPort::Data8);//to set the word length
        serial->setParity(QSerialPort::NoParity);
        serial->setStopBits(QSerialPort::OneStop);
        serial->setFlowControl(QSerialPort::NoFlowControl);
    }

    itoa(widthSpinBox->value(),buffer,10);

    statusLabel->setText(buffer);

    serial->write(buffer);
    serial->putChar('\r');



    //frequency
    itoa(freqSpinBox->value(),buffer,10);
    serial->write(buffer);
    serial->putChar('\r');

    char dig = (char)(((int)'0')+phasePortComboBox->currentIndex());
    serial->putChar(dig);
    serial->putChar('\r');




    statusLabel->setText("Ready to go!");
    //serial->close();



}

void Dialog::convertBuffer(int data)
{
    int unita;
    int decina;
    int centinaia;

    unita = data % 10;
    decina = (data % 100)-unita;
    centinaia = (data-(unita+decina))/100;

    buffer[2] = (char)(((int)'0')+unita);
    buffer[1] = (char)(((int)'0')+decina);
    buffer[0] = (char)(((int)'0')+centinaia);

}

void Dialog::openSerial(void){
    if(!status)
    {
        int index = serialPortComboBox->currentIndex();
        status= true;
        serial->setPortName(QSPInfo[index].portName());
        serial->open(QIODevice::ReadWrite);
        serial->setBaudRate(QSerialPort::Baud9600); // to set the wished Baudrate
        serial->setDataBits(QSerialPort::Data8);//to set the word length
        serial->setParity(QSerialPort::NoParity);
        serial->setStopBits(QSerialPort::OneStop);
        serial->setFlowControl(QSerialPort::NoFlowControl);
        connect(serial,SIGNAL(readyRead()), this,SLOT(serialReceived()));
        //openButton->setText("Close");
    }
    else
    {
        status = false;
        serial->close();
        //openButton->setText("Open");
    }

}


void Dialog::serialReceived( )
{
    QString ba;
    ba=serial->readAll();
    buso +=ba;
    trafficLabel->setText(buso);
}

char* Dialog::itoa(int value, char* result, int base) {
        // check that the base if valid
        if (base < 2 || base > 36) { *result = '\0'; return result; }

        char* ptr = result, *ptr1 = result, tmp_char;
        int tmp_value;

        do {
            tmp_value = value;
            value /= base;
            *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
        } while ( value );

        // Apply negative sign
        if (tmp_value < 0) *ptr++ = '-';
        *ptr-- = '\0';
        while(ptr1 < ptr) {
            tmp_char = *ptr;
            *ptr--= *ptr1;
            *ptr1++ = tmp_char;
        }
        return result;
}

void Dialog::updatePorts()
{
    serialPortComboBox->clear();
    int i= 0;

        foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
        {
            int x = QString::compare(info.manufacturer(), "Texas Instruments", Qt::CaseInsensitive);
            int y = QString::compare(info.manufacturer(), "TI", Qt::CaseInsensitive);
            if((x&y) == 0)
            {
                serialPortComboBox->addItem(info.description()+" - "+info.portName());
                QSPInfo[i] = info;
                i++;
            }
        }
        timer->start(1000);
}
