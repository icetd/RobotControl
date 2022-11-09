#ifndef ADDTOPICPAGE_H
#define ADDTOPICPAGE_H

#include <QWidget>
#include <QTreeWidgetItem>
#include <QCheckBox>

namespace Ui {
class AddTopicPage;
}

class AddTopicPage : public QWidget
{
    Q_OBJECT

public:
    explicit AddTopicPage(QWidget *parent = NULL);
    ~AddTopicPage();
    QTreeWidgetItem *choose;

signals:
    void topicChoose(QTreeWidgetItem *choose, QString name);

private slots:
    void btn_cancel_clicked();
    void btn_ok_clicked();
    void slots_currentItem_changed(QTreeWidgetItem *current, QTreeWidgetItem *previous);

private:
    Ui::AddTopicPage *ui;
    void initUi();
    void connects();
    QTreeWidgetItem *Navigate=NULL;
    QTreeWidgetItem *Build_Map=NULL;
    QTreeWidgetItem *Axes=NULL;
    QTreeWidgetItem *Camera=NULL;
    QTreeWidgetItem *DepthCloud=NULL;
    QTreeWidgetItem *Effort=NULL;
    QTreeWidgetItem *FluidPressure=NULL;
    QTreeWidgetItem *Grid=NULL;
    QTreeWidgetItem *GridCells=NULL;
    QTreeWidgetItem *Group=NULL;
    QTreeWidgetItem *Illuminance=NULL;
    QTreeWidgetItem *Image=NULL;
    QTreeWidgetItem *InteractiveMarkers=NULL;
    QTreeWidgetItem *LaserScan=NULL;
    QTreeWidgetItem *Map=NULL;
    QTreeWidgetItem *Marker=NULL;
    QTreeWidgetItem *MarkerArray=NULL;
    QTreeWidgetItem *Odometry=NULL;
    QTreeWidgetItem *Path=NULL;
    QTreeWidgetItem *PointCloud=NULL;
    QTreeWidgetItem *PointCloud2=NULL;
    QTreeWidgetItem *PointStamped=NULL;
    QTreeWidgetItem *Polygon=NULL;
    QTreeWidgetItem *Pose=NULL;
    QTreeWidgetItem *PoseArray=NULL;
    QTreeWidgetItem *PoseWithCovariance=NULL;
    QTreeWidgetItem *Range=NULL;
    QTreeWidgetItem *RelativeHumidity=NULL;
    QTreeWidgetItem *RobotModel=NULL;
    QTreeWidgetItem *TF=NULL;
    QTreeWidgetItem *Temperature=NULL;
    QTreeWidgetItem *WrenchStamped=NULL;
    QTreeWidgetItem *Imu=NULL;
    QCheckBox *checkbox;
};

#endif // ADDTOPICPAGE_H
