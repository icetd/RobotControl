#include <QTabWidget>
#include <QDebug>
#include <QFile>
#include <QIcon>
#include <QMessageBox>
#include <QLineEdit>
#include <QInputDialog>
#include <QFileDialog>
#include <QStandardPaths>
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("RobotControl");
    this->setWindowIcon(QIcon(":/Icon/Mario.ico"));
    
    controlWidget = new ControlPage;
    app_node = new AppNode(argc, argv);
    ui->tabWidget_right->addTab(controlWidget, "控制");
    
    ui->tabWidget_left->setCurrentIndex(0);

    add_topic = new AddTopicPage();

    initData();
    connections(); 
    readConfigs();
 
}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::readConfigs()
{

}

void MainWindow::initAll()
{
}

void MainWindow::initRviz()
{
    ui->label_rviz->hide();
    app_rviz = new AppRviz(ui->layout_rviz, "AppRviz");
    connect(app_rviz, &AppRviz::returnModeSignal, this, &MainWindow::rvizGetModel);
    app_rviz->getDisplayTreeMode();
    QMap<QString, QVariant> nameValue;
    nameValue.insert("Line Style", "Billboards");
    nameValue.insert("Color", QColor(160, 160, 160));
    nameValue.insert("Plane Cell Count", 30);
    app_rviz->displayInit(RVIZ_DISPLAY_GRID, "Grid", true, nameValue); 
    QString path = "/opt/RobotControl/config/rviz.yaml";
    app_rviz->ReadDisplaySet(path);
}

void MainWindow::initData()
{
    m_mapRvizDisplays.insert("Axes", RVIZ_DISPLAY_AXES);
    m_mapRvizDisplays.insert("Camera", RVIZ_DISPLAY_CAMERA);
    m_mapRvizDisplays.insert("DepthCloud", RVIZ_DISPLAY_DEPTHCLOUD);
    m_mapRvizDisplays.insert("Effort", RVIZ_DISPLAY_EFFORT);
    m_mapRvizDisplays.insert("FluidPressure", RVIZ_DISPLAY_FLUIDPRESSURE);
    m_mapRvizDisplays.insert("Grid", RVIZ_DISPLAY_GRID);
    m_mapRvizDisplays.insert("GridCells", RVIZ_DISPLAY_GRIDCELLS);
    m_mapRvizDisplays.insert("Group", RVIZ_DISPLAY_GROUP);
    m_mapRvizDisplays.insert("Illuminance", RVIZ_DISPLAY_ILLUMINANCE);
    m_mapRvizDisplays.insert("Image", RVIZ_DISPLAY_IMAGE);
    m_mapRvizDisplays.insert("InterativerMarker", RVIZ_DISPLAY_INTERATIVEMARKER);
    m_mapRvizDisplays.insert("LaserScan", RVIZ_DISPLAY_LASERSCAN);
    m_mapRvizDisplays.insert("Map", RVIZ_DISPLAY_MAP);
    m_mapRvizDisplays.insert("Marker", RVIZ_DISPLAY_MARKER);
    m_mapRvizDisplays.insert("MarkerArray", RVIZ_DISPLAY_MARKERARRAY);
    m_mapRvizDisplays.insert("Odometry", RVIZ_DISPLAY_ODOMETRY);
    m_mapRvizDisplays.insert("Path", RVIZ_DISPLAY_PATH);
    m_mapRvizDisplays.insert("PointCloud", RVIZ_DISPLAY_POINTCLOUD);
    m_mapRvizDisplays.insert("PointCloud2", RVIZ_DISPLAY_POINTCLOUD2);
    m_mapRvizDisplays.insert("PointStamped", RVIZ_DISPLAY_POINTSTAMPED);
    m_mapRvizDisplays.insert("Polygon", RVIZ_DISPLAY_POLYGON);
    m_mapRvizDisplays.insert("Pose", RVIZ_DISPLAY_POSE);
    m_mapRvizDisplays.insert("PoseArray", RVIZ_DISPLAY_POSEARRAY);
    m_mapRvizDisplays.insert("PoseWithCovariance", RVIZ_DISPLAY_POSEWITHCOVARIANCE);
    m_mapRvizDisplays.insert("Range", RVIZ_DISPLAY_RANGE);
    m_mapRvizDisplays.insert("RelativeHumidity", RVIZ_DISPLAY_RELATIVEHUMIDITY);
    m_mapRvizDisplays.insert("RobotModel", RVIZ_DISPLAY_ROBOTMODEL);
    m_mapRvizDisplays.insert("TF", RVIZ_DISPLAY_TF);
    m_mapRvizDisplays.insert("Temperature", RVIZ_DISPLAY_TEMPERATURE);
    m_mapRvizDisplays.insert("WrenchStamped", RVIZ_DISPLAY_WRENCHSTAMPED);   
}

void MainWindow::connections()
{
    connect(controlWidget, SIGNAL(signal_send_cmd(char, int16_t, int16_t)), this, SLOT(slot_cmd_control(char, int16_t, int16_t)));
    connect(add_topic, SIGNAL(topicChoose(QTreeWidgetItem *, QString)), this, SLOT(slot_choose_topic(QTreeWidgetItem *, QString)));
    connect(ui->btn_add_topic, SIGNAL(clicked()), this, SLOT(btn_add_topic_clicked()));
    connect(ui->btn_remove_topic, SIGNAL(clicked()), this, SLOT(btn_remove_topic_clicked()));
    connect(ui->btn_rename_topic, SIGNAL(clicked()), this, SLOT(btn_rename_topic_clicked()));
    connect(ui->btn_load_rvizConfig, SIGNAL(clicked()), this, SLOT(btn_load_rvizConfig_clicked()));
    connect(ui->btn_save_rvizConfig, SIGNAL(clicked()), this, SLOT(btn_save_rvizConfig_clicked()));
    connect(ui->btn_disconnect, SIGNAL(clicked()), this, SLOT(slot_disConenct()));
    connect(app_node, SIGNAL(rosShutdown()), this, SLOT(slot_disConenct()));
    connect(app_node, SIGNAL(speed_x(double)), this, SLOT(slot_speed_x(double)));
    connect(app_node, SIGNAL(speed_y(double)), this, SLOT(slot_speed_y(double)));
    connect(app_node, SIGNAL(signal_position(QString, double, double, double, double)), this, 
                SLOT(slot_position_change(QString, double, double, double, double)));
    connect(ui->btn_set_pos, SIGNAL(clicked()), this, SLOT(slot_set_2D_Pos()));
    connect(ui->btn_set_goal, SIGNAL(clicked()), this, SLOT(slot_set_2D_Goal()));
    connect(ui->btn_move_camera, SIGNAL(clicked()), this, SLOT(slot_move_camera()));
    connect(ui->btn_select, SIGNAL(clicked()), this, SLOT(slot_set_select()));
    connect(ui->btn_set_return, SIGNAL(clicked()), this, SLOT(slot_set_return_point()));
    connect(ui->btn_return, SIGNAL(clicked()), this, SLOT(slot_return_point()));
}

bool MainWindow::connectMaster(QString master_ip, QString local_ip, bool use_env)
{
    if (use_env) {
        if (!app_node->init()) {
            return false;
        } else {
            
        }
    } else {
        if (!app_node->init(master_ip.toStdString(), local_ip.toStdString())) {
            return false;
        } else {
            QFile qss(":/Qss/style1.qss");
            qss.open(QFile::ReadOnly);
            qApp->setStyleSheet(qss.readAll());
            qss.close();
            initRviz();
        }
    }
    return true;
}

QString MainWindow::judgeDisplayNewName(QString name)
{
    if (m_modelRvizDisplay != nullptr)
    {
        bool isSame = true;
        while (isSame)
        {
            isSame = false;
            for (int i = 0; i < m_modelRvizDisplay->rowCount(); i++) {
                if (m_modelRvizDisplay->index(i, 0).data().value<QString>() == name) {
                    if (name.indexOf("_") != -1) {
                        int num = name.section("_", -1, -1).toInt();
                        name = name.left(name.length() - name.section("_", -1, -1).length() - 1);
                        if (num <= 1) {
                            num = 2;
                        } else {
                            num++;
                        }
                        name = name + "_" + QString::number(num);
                    } else {
                        name = name + "_2";
                    }
                    isSame = true;
                    break;
                }
            }
        }
    }
    return name;
}

void MainWindow::slot_cmd_control(char cmd, int16_t speed_liner, int16_t speed_angle)
{
    float liner = speed_liner * 0.01;
    float angle = speed_angle * 0.01;
    app_node->move(cmd, liner, angle);
}

void MainWindow::slot_choose_topic(QTreeWidgetItem* choose, QString name)
{
    QString classId = choose->text(0);
    
    qDebug() << name;

    name = judgeDisplayNewName(name);

    qDebug() << name;
    QMap<QString, QVariant> namevalue;
    namevalue.clear();
    app_rviz->displayInit(m_mapRvizDisplays[classId], name, true, namevalue);
}

void MainWindow::rvizGetModel(QAbstractItemModel *model) 
{
    m_modelRvizDisplay = model;
    ui->treeView_rviz->setModel(model);
}

void MainWindow::btn_add_topic_clicked()
{
    add_topic->show();
    ui->btn_remove_topic->setEnabled(true);
    ui->btn_rename_topic->setEnabled(true);
}

void MainWindow::btn_remove_topic_clicked()
{
    if(ui->treeView_rviz->currentIndex().row() >= 0) {
        m_sRvizDisplayChooseName = ui->treeView_rviz->currentIndex().data().value<QString>();
        app_rviz->removeDisplay(m_sRvizDisplayChooseName);
        if (ui->treeView_rviz->currentIndex().row() >= 0) {
            on_treeView_rvizDisplayTree_clicked(ui->treeView_rviz->currentIndex());
        } else {
            m_sRvizDisplayChooseName.clear();
        }
    } else {
        inform("请选择Display后在执行此操作");
    }
}

void MainWindow::btn_rename_topic_clicked()
{
    if (ui->treeView_rviz->currentIndex().row() < 0)
    {
        inform("请选择Display后再执行此操作");
        return ;
    }
    m_sRvizDisplayChooseName = ui->treeView_rviz->currentIndex().data().value<QString>();
    QString dlgTitle = "重命名";
    QString txtlabel = "请输入名字：";
    QString defaultInupt = m_sRvizDisplayChooseName;
    QLineEdit::EchoMode echoMode = QLineEdit::Normal;
    bool ok = false;
    QString newname = QInputDialog::getText(this, dlgTitle, txtlabel, echoMode, defaultInupt, &ok);
    if (ok && !newname.isEmpty())
    {
        if (newname != defaultInupt)
        {
            QString nosamename = judgeDisplayNewName(newname);
            app_rviz->renameDisplay(defaultInupt, nosamename);
            m_sRvizDisplayChooseName = nosamename;
            if (nosamename != newname)
            {
                inform("命名重复！命名已自动更改为" + nosamename);
            }
        }
    }
    else if (ok)
    {
        inform("输入内容为空，重命名失败");
    }
}

void MainWindow::btn_load_rvizConfig_clicked()
{
    if (app_rviz == nullptr) {
        return;
    }
    QString path = QFileDialog::getOpenFileName(this, "导入 RVIZ 配置", "/opt/RobotControl/config/", "YAML(*.yaml);;All(*.*)");
    if (!path.isEmpty()) {
        qDebug() << path;
        app_rviz->ReadDisplaySet(path);
    }
}

void MainWindow::btn_save_rvizConfig_clicked()
{
    if (app_rviz == nullptr) {
        return;
    }
    QString path = QFileDialog::getSaveFileName(this, "导出 RVIZ 配置", "/opt/RobotControl/config/", "YAML(*.yaml);;All(*.*)");

    if (!path.isEmpty()) {
        if (path.section('/', -1, -1).indexOf('.') < 0) {
            path = path + ".yaml";
        }
        app_rviz->outDisplaySet(path);
    }
}


void MainWindow::inform(QString str) 
{
    QMessageBox m_r;
    m_r.setWindowTitle("提示");
    m_r.setText(str);
    m_r.addButton(tr("确定"), QMessageBox::ActionRole);
    m_r.addButton(tr("取消"), QMessageBox::ActionRole);
}

void MainWindow::on_treeView_rvizDisplayTree_clicked(const QModelIndex &index)
{
    m_sRvizDisplayChooseName = index.data().value<QString>();
    if (index.parent().row() == -1) {
        if (index.row() > 1) {
            ui->btn_remove_topic->setEnabled(true);
            ui->btn_rename_topic->setEnabled(true);
        } else {
            ui->btn_remove_topic->setEnabled(false);
            ui->btn_rename_topic->setEnabled(false);
        }   
    } else {
            ui->btn_remove_topic->setEnabled(false);
            ui->btn_rename_topic->setEnabled(false);
    }
}

QString MainWindow::getUsername()
{
    QString userPath = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
    QString userName = userPath.section("/", -1, -1);
    return userName;
}

 void MainWindow::slot_disConenct()
 {
    ros::shutdown();
    slot_rosShutdown();
    emit signalDisconnect();
    this->close();
 }

 void MainWindow::slot_rosShutdown()
 {
    slot_updateRobotStatus(RobotStatus::none);
 }

 void MainWindow::slot_updateRobotStatus(RobotStatus status)
 {
    switch (status) {
    case RobotStatus::none: 
        break;
    }
 }

 void MainWindow::slot_speed_x(double x)
 {
 }
 void MainWindow::slot_speed_y(double y)
 {
 }
 void MainWindow::cmd_output()
 {

 }

 void MainWindow::slot_set_2D_Goal()
 {
    app_rviz->setGoal();
 }
 void MainWindow::slot_set_2D_Pos()
 {
    app_rviz->setPos();
 }
 void MainWindow::slot_set_select()
 {
    app_rviz->setSelect();
 }
 void MainWindow::slot_move_camera()
 {
    app_rviz->setMoveCamera();
 }

 void MainWindow::slot_set_return_point()
 {
    app_node->set_goal(ui->label_frame->text(), ui->label_x_return->text().toDouble(), ui->label_y_return->text().toDouble(),
                                                ui->label_z_return->text().toDouble(), ui->label_w_return->text().toDouble());
 }
 void MainWindow::slot_return_point()
 {
 }
 void MainWindow::slot_position_change(QString frame, double x, double y, double z, double w)
 {
    ui->label_frame->setText(frame);
    ui->label_x->setText(QString::number(x));
    ui->label_y->setText(QString::number(y));
    ui->label_z->setText(QString::number(z));
    ui->label_w->setText(QString::number(w));
 }