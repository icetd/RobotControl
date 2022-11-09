#ifndef LOGINPAGE_H
#define LOGINPAGE_H

#include <QWidget>
#include "mainwindow.h"

namespace Ui {
    class LoginPage;
}

class LoginPage : public QWidget
{
    Q_OBJECT

public:
    explicit LoginPage(int argc, char **argv, QWidget *parent = nullptr);
    ~LoginPage();

private slots:
    void btn_setting_clicked();
    void btn_return_main_clicked();
    void btn_save_config();
    void btn_connect_clicked();
    void slot_showLogin(); 

private:
    Ui::LoginPage *ui = nullptr;
    MainWindow *mainwindow = nullptr;
    bool m_autoLogin = false;
    int m_argc;
    char **m_argv;
    void initWidget();
    void readConfig();
    void saveConfig();
    void Login();


};


#endif // LOGINPAGE_H
