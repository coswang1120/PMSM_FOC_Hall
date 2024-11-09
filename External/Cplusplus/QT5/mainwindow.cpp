#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , serialPort(new QSerialPort(this))
    , receiveBuffer("")
{
    ui->setupUi(this);
    
    // 初始化串口
    setupSerialPort();
    
    // 更新可用串口列表
    updatePortList();
    
    // 連接信號和槽
    connect(serialPort, &QSerialPort::readyRead, this, &MainWindow::handleSerialDataReceived);
}

MainWindow::~MainWindow()
{
    if (serialPort->isOpen()) {
        serialPort->close();
    }
    delete ui;
}

void MainWindow::setupSerialPort()
{
    serialPort->setBaudRate(QSerialPort::Baud115200);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setParity(QSerialPort::NoParity);
}

void MainWindow::updatePortList()
{
    ui->portComboBox->clear();
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos) {
        ui->portComboBox->addItem(info.portName());
    }
}

void MainWindow::on_connectButton_clicked()
{
    if (!serialPort->isOpen()) {
        serialPort->setPortName(ui->portComboBox->currentText());
        if (serialPort->open(QIODevice::ReadWrite)) {
            ui->connectButton->setText("Disconnect");
            ui->portComboBox->setEnabled(false);
        } else {            
            QMessageBox::warning(this, "警告", "串口未開啟！");

        }
    } else {
        serialPort->close();
        ui->connectButton->setText("Connect");
        ui->portComboBox->setEnabled(true);
    }
}

void MainWindow::handleSerialDataReceived()
{
    const QByteArray data = serialPort->readAll();
    receiveBuffer += QString::fromUtf8(data);
    
    // 檢查是否有完整的行
    while (receiveBuffer.contains('\n') || receiveBuffer.contains('\r')) {
        int endIndex = receiveBuffer.indexOf('\n');
        if (endIndex == -1) {
            endIndex = receiveBuffer.indexOf('\r');
        }
        
        if (endIndex >= 0) {
            // 提取完整的行
            QString line = receiveBuffer.left(endIndex).trimmed();
            if (!line.isEmpty()) {
                ui->upperReceiveText->moveCursor(QTextCursor::End);
                ui->upperReceiveText->insertPlainText(line + "\n");
            }
            // 移除已處理的數據
            receiveBuffer = receiveBuffer.mid(endIndex + 1);
        }
    }
}

void MainWindow::sendData(const QString &data)
{
    if (serialPort->isOpen()) {
        serialPort->write((data + "\r\n").toUtf8());
        ui->lowerSendText->append(data);
    } else {
        QMessageBox::warning(this, tr("警告"), tr("串口未開啟！"));
    }
}

void MainWindow::on_clearReceiveButton_clicked()
{
    ui->upperReceiveText->clear();
}

void MainWindow::on_clearSendButton_clicked()
{
    ui->lowerSendText->clear();
}

void MainWindow::on_sendButton_clicked()
{
    QString data = ui->sendLineEdit->text();
    if (!data.isEmpty()) {
        sendData(data);
        ui->sendLineEdit->clear();
    }
}

void MainWindow::on_motorSet0Button_clicked()
{
    sendData("Motor set 0");
}

void MainWindow::on_motorSet1Button_clicked()
{
    sendData("Motor set 1");
}

void MainWindow::on_motorSet2Button_clicked()
{
    sendData("Motor set 2");
}
