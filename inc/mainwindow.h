#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/qmainwindow.h>
#include <QMap>
#include <QTreeWidgetItem>
#include <QModelIndex>
#include "ControlPage.h"
#include "AppNode.h"
#include "AppRviz.h"
#include "AddTopicPage.h"
#include "RobotBase.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char **argv, QWidget *parent = nullptr);
    ~MainWindow();

    bool connectMaster(QString master_ip, QString local_ip, bool use_env);

signals:
    void signalDisconnect();

public slots:
    void rvizGetModel(QAbstractItemModel *model);
    void slot_choose_topic(QTreeWidgetItem* choose, QString name);
    void slot_rosShutdown();
    void slot_updateRobotStatus(RobotStatus status);

    void btn_add_topic_clicked();
    void btn_remove_topic_clicked();
    void btn_rename_topic_clicked();
    void btn_load_rvizConfig_clicked();
    void btn_save_rvizConfig_clicked();
    void slot_disConenct();
    void slot_speed_x(double x);
    void slot_speed_y(double y);
    void cmd_output();

    void slot_set_2D_Goal();
    void slot_set_2D_Pos();
    void slot_set_select();
    void slot_move_camera();

    void slot_set_return_point();
    void slot_return_point();
    void slot_position_change(QString frame, double x, double y, double z, double w);

    void on_treeView_rvizDisplayTree_clicked(const QModelIndex &index);
private slots:
    void slot_cmd_control(char cmd, int16_t speed_liner, int16_t speed_angle);


private:
    Ui::MainWindow *ui;
    ControlPage *controlWidget;
    int m_currentSilderPage;

    AppNode *app_node;
    
    AppRviz *app_rviz = nullptr;
    AddTopicPage *add_topic = nullptr;

    QMap<QWidget*, QTreeWidgetItem*> widtget_to_parentIten_map;
    QMap<QString, QTreeWidgetItem *> tree_rviz_status;
    QMap<QTreeWidgetItem*, QMap<QString, QString>> tree_rviz_valuse;


    QAbstractItemModel *m_modelRvizDisplay;
    QMap<QString, QString> m_mapRvizDisplays;
    QString m_sRvizDisplayChooseName;

    void readConfigs();
    void initAll();
    void initData();
    QString judgeDisplayNewName(QString name);
    void initRviz();
    void connections();
    void inform(QString str);
    QString getUsername();
    
};
#endif // MAINWINDOW_H
