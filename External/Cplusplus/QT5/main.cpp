#include "mainwindow.h"
#include <QApplication>
#include <QTextCodec>
#include <QFont>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    
    // 設定全局編碼為 UTF-8
    #if (QT_VERSION <= QT_VERSION_CHECK(5,0,0))
        QTextCodec::setCodecForTr(QTextCodec::codecForName("UTF-8"));
        QTextCodec::setCodecForCStrings(QTextCodec::codecForName("UTF-8"));
    #endif
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
    
    // 設定全局字體
    QFont font("Microsoft YaHei", 9);  // 使用微軟雅黑字體
    a.setFont(font);
    
    MainWindow w;
    w.show();
    return a.exec();
} 