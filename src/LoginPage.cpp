#include "LoginPage.h"
#include "ui_LoginPage.h"
#include "mainwindow.h"
#include <QFile>
#include <QDebug>
#include <QSettings>
#include <QMessageBox>
#include <QDesktopWidget>


LoginPage::LoginPage(int argc, char **argv, QWidget *parent) :
    m_argc (argc),
    m_argv (argv),
    QWidget(parent),
    ui(new Ui::LoginPage)
{

    QFile qss(":/Qss/style1.qss");
    qss.open(QFile::ReadOnly);
    qApp->setStyleSheet(qss.readAll());
    qss.close();

    ui->setupUi(this);
    this->setWindowTitle("RobotControl");
    this->setWindowIcon(QIcon(":/Icon/Mario.ico"));

    initWidget();
    readConfig();
}

LoginPage::~LoginPage()
{
    delete ui;
}

void LoginPage::initWidget()
{
    ui->stackedWidget->setCurrentIndex(0);
    connect(ui->btn_setting, SIGNAL(clicked()), this, SLOT(btn_setting_clicked()));
    connect(ui->btn_return_main, SIGNAL(clicked()), this, SLOT(btn_return_main_clicked()));
    connect(ui->btn_save_config, SIGNAL(clicked()), this, SLOT(btn_save_config()));
    connect(ui->btn_connect, SIGNAL(clicked()), this, SLOT(btn_connect_clicked()));

}

void LoginPage::readConfig()
{
    QString topic_speed;
    QString topic_odom;
    QString topic_pose;
    QString topic_goal;
 
    QString frame_laser;
    QString frame_map;
    QString frame_base;

    QString ros_threadNum;
    QString ros_rate;

    QString ros_master_url;
    QString ros_local_host;
    
    QSettings* configs = new QSettings("/opt/RobotControl/config/config.ini", QSettings::IniFormat);   

    topic_speed = configs->value("topic/topic_speed").toString();
    topic_odom = configs->value("topic/topic_odom").toString();
    topic_pose = configs->value("topic/topic_pose").toString();
    topic_goal = configs->value("topic/topic_goal").toString();
    
    frame_laser = configs->value("frame/frame_laser").toString();
    frame_map = configs->value("frame/frame_map").toString();
    frame_base = configs->value("frame/frame_base").toString();

    ros_threadNum = configs->value("ros/thread_num").toString();
    ros_rate = configs->value("ros/rate").toString();

    ros_master_url = configs->value("ros/master_url").toString();
    ros_local_host = configs->value("ros/local_ip").toString();

    ui->lEdit_Topic_speed->setText(topic_speed);
    ui->lEdit_Topic_odom->setText(topic_odom);
    ui->lEdit_Topic_pose->setText(topic_pose);
    ui->lEdit_Topic_goal->setText(topic_goal);

    ui->lEdit_Frame_laser->setText(frame_laser);
    ui->lEdit_Frame_map->setText(frame_map);
    ui->lEdit_Frame_robotBase->setText(frame_base);

    ui->sb_thread->setValue(ros_threadNum.toInt());
    ui->sb_rate->setValue(ros_rate.toInt());

    ui->lEdit_master_url->setText(ros_master_url);
    ui->lEdit_local_ip->setText(ros_local_host);
}

void LoginPage::saveConfig()
{
    QSettings* configs = new QSettings("/opt/RobotControl/config/config.ini", QSettings::IniFormat);
    configs->setValue("topic/topic_speed", ui->lEdit_Topic_speed->text());
    configs->setValue("topic/topic_odom", ui->lEdit_Topic_odom->text());
    configs->setValue("topic/topic_pose", ui->lEdit_Topic_pose->text());
    configs->setValue("topic/topic_goal", ui->lEdit_Topic_goal->text());

    configs->setValue("frame/frame_laser", ui->lEdit_Frame_laser->text());
    configs->setValue("frame/frame_map", ui->lEdit_Frame_map->text());
    configs->setValue("frame/frame_base", ui->lEdit_Frame_robotBase->text());

    configs->setValue("ros/thread_num", ui->sb_thread->text());
    configs->setValue("ros/rate", ui->sb_rate->text());
    configs->setValue("ros/master_url", ui->lEdit_master_url->text());
    configs->setValue("ros/local_ip", ui->lEdit_local_ip->text());
}

void LoginPage::Login()
{
    if (mainwindow != nullptr) 
        mainwindow->close();       
    mainwindow = new MainWindow(m_argc, m_argv);
    connect(mainwindow, SIGNAL(signalDisconnect()), this, SLOT(slot_showLogin()));

    QCoreApplication::processEvents();

    ui->btn_connect->setText("取消连接");
    QString master_url;
    QString local_ip;
    master_url = ui->lEdit_master_url->text();
    local_ip = ui->lEdit_local_ip->text();

    bool isConencted = mainwindow->connectMaster(master_url, local_ip, false);

    if (isConencted) {
        QSettings *configs = new QSettings("/opt/RobotControl/config/config.ini", QSettings::IniFormat);
        configs->setValue("ros/master_url", ui->lEdit_master_url->text());
        configs->setValue("ros/locat_ip", ui->lEdit_local_ip->text());

        ui->btn_connect->setText("连接");
        this->hide();
        mainwindow->show();
        mainwindow->move(qApp->desktop()->width()/2 - mainwindow->width()/2 ,
        qApp->desktop()->height() / 2 - mainwindow->height() / 2);

    } else {
        ui->btn_connect->setText("连接");
        QMessageBox::information(NULL, "连接失败", "连接失败！请检查配置", QMessageBox::Yes);
    }
}

void LoginPage::btn_setting_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);
}

void LoginPage::btn_return_main_clicked()
{
    ui->stackedWidget->setCurrentIndex(0);
}

void LoginPage::btn_save_config()
{
    saveConfig();
}

void LoginPage::btn_connect_clicked()
{
    if (ui->btn_connect->text() != "取消") {
        Login();
    } else {
        ui->btn_connect->setText("连接");
        m_autoLogin = false;
    }
}

void LoginPage::slot_showLogin()
{
    this->show();
    ui->btn_connect->setEnabled(true);
}