#include "ControlPage.h"
#include "ui_ControlPage.h"
#include <QDebug>

ControlPage::ControlPage(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControlPage)
{

    ui->setupUi(this);
    connect(ui->btn_left_forward, SIGNAL(clicked()), SLOT(btn_left_forward_clicked()));
    connect(ui->btn_forward, SIGNAL(clicked()), SLOT(btn_forward_clicked()));
    connect(ui->btn_right_forward, SIGNAL(clicked()), SLOT(btn_right_forward_clicked()));
    connect(ui->btn_left, SIGNAL(clicked()), SLOT(btn_left_clicked()));
    connect(ui->btn_stop, SIGNAL(clicked()), SLOT(btn_stop_clicked()));
    connect(ui->btn_right, SIGNAL(clicked()), SLOT(btn_right_clicked()));
    connect(ui->btn_left_back, SIGNAL(clicked()), SLOT(btn_left_back_clicked()));
    connect(ui->btn_back, SIGNAL(clicked()), SLOT(btn_back_clicked()));
    connect(ui->btn_right_back, SIGNAL(clicked()), SLOT(btn_right_back_clicked()));
    connect(ui->hzs_speed, SIGNAL(valueChanged(int)), this, SLOT(hzs_speed_moved(int)));
    connect(ui->hzs_angle_speed, SIGNAL(valueChanged(int)), this, SLOT(hzs_angle_speed_moved(int)));
}


ControlPage::~ControlPage()
{
    delete ui;
}

void ControlPage::sendCmd()
{
    int16_t speed_liner = ui->hzs_speed->value();
    int16_t speed_angle = ui->hzs_angle_speed->value();
    emit signal_send_cmd(m_cmd, speed_liner, speed_angle);
}
void ControlPage::hzs_speed_moved(int value)
{

    int16_t speed_liner = ui->hzs_speed->value();
    ui->lcd_speed->display(speed_liner);
}
void ControlPage::hzs_angle_speed_moved(int value)
{
    int16_t speed_angle = ui->hzs_angle_speed->value();
    ui->lcd_angle_speed->display(speed_angle);
}

void ControlPage::btn_left_forward_clicked()
{
    m_cmd = 'u';
    sendCmd();
}

void ControlPage::btn_forward_clicked()
{
    m_cmd = 'i';
    sendCmd();
}

void ControlPage::btn_right_forward_clicked()
{
    m_cmd = 'o';
    sendCmd();
}

void ControlPage::btn_left_clicked()
{
    m_cmd = 'j';
    sendCmd();
}

void ControlPage::btn_stop_clicked()
{
    m_cmd = 'k';
    sendCmd();
}

void ControlPage::btn_right_clicked()
{
    m_cmd = 'l';
    sendCmd();
}

void ControlPage::btn_left_back_clicked()
{
    m_cmd = 'm';
    sendCmd();
}

void ControlPage::btn_back_clicked()
{
    m_cmd = ',';
    sendCmd();
}

void ControlPage::btn_right_back_clicked()
{
    m_cmd = '.';
    sendCmd();
}
