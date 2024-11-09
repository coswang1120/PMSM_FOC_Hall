#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_connectButton_clicked();
    void on_clearReceiveButton_clicked();
    void on_clearSendButton_clicked();
    void on_sendButton_clicked();
    void on_motorSet0Button_clicked();
    void on_motorSet1Button_clicked();
    void on_motorSet2Button_clicked();
    void handleSerialDataReceived();
    void updatePortList();

private:
    Ui::MainWindow *ui;
    QSerialPort *serialPort;
    QString receiveBuffer;
    void sendData(const QString &data);
    void setupSerialPort();
};
#endif // MAINWINDOW_H 