#ifndef CONTROLPAGE_H
#define CONTROLPAGE_H

#include <QWidget>

namespace Ui {
    class ControlPage;
}

class ControlPage : public QWidget
{
    Q_OBJECT

public:
    explicit ControlPage(QWidget *parent = nullptr);
    ~ControlPage();

signals:
    void signal_send_cmd(char cmd, int16_t _speed_liner, int16_t _speed_angle);

private slots:
    void btn_left_forward_clicked();
    void btn_forward_clicked();
    void btn_right_forward_clicked();
    void btn_left_clicked();
    void btn_stop_clicked();
    void btn_right_clicked();
    void btn_left_back_clicked();
    void btn_back_clicked();
    void btn_right_back_clicked();
    void hzs_speed_moved(int value);
    void hzs_angle_speed_moved(int value);

private:
    Ui::ControlPage *ui;

    char m_cmd;
    void sendCmd();
};

#endif // CONTROLPAGE_H
