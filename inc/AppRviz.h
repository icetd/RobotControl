#ifndef APP_RVIZ_H
#define APP_RVIZ_H

#include <QVBoxLayout>
#include <QMap>
#include <QColor>
#include <QObject>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <rviz/image/ros_image_texture.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/yaml_config_reader.h>
#include <rviz/yaml_config_writer.h>
#include <rviz/display_group.h>

#define RVIZ_DISPLAY_AXES               "rviz/Axes"
#define RVIZ_DISPLAY_CAMERA             "rviz/Camera"
#define RVIZ_DISPLAY_DEPTHCLOUD         "rviz/DepthCloud"
#define RVIZ_DISPLAY_EFFORT             "rviz/Errort"
#define RVIZ_DISPLAY_FLUIDPRESSURE      "rviz/Fluidpressure"
#define RVIZ_DISPLAY_GRID               "rviz/Grid"
#define RVIZ_DISPLAY_GRIDCELLS          "rvize/GridCells"
#define RVIZ_DISPLAY_GROUP              "rviz/Group"
#define RVIZ_DISPLAY_ILLUMINANCE        "rviz/Illuminance"
#define RVIZ_DISPLAY_IMAGE              "rviz/Image"
#define RVIZ_DISPLAY_INTERATIVEMARKER   "rviz/InteractiveMarker"
#define RVIZ_DISPLAY_LASERSCAN          "rviz/LaserScan"
#define RVIZ_DISPLAY_MAP                "rviz/Map"
#define RVIZ_DISPLAY_MARKER             "rviz/Marker"
#define RVIZ_DISPLAY_MARKERARRAY        "rviz/MarkerArray"
#define RVIZ_DISPLAY_ODOMETRY           "rviz/Odometry"
#define RVIZ_DISPLAY_PATH               "rviz/Path"
#define RVIZ_DISPLAY_POINTCLOUD         "rviz/PointCloud"
#define RVIZ_DISPLAY_POINTCLOUD2        "rviz/PointCloud2"
#define RVIZ_DISPLAY_POINTSTAMPED       "rviz/PointStamped"
#define RVIZ_DISPLAY_POLYGON            "rviz/Polygon"
#define RVIZ_DISPLAY_POSE               "rviz/Pose"
#define RVIZ_DISPLAY_POSEARRAY          "rviz/PoseArray"
#define RVIZ_DISPLAY_POSEWITHCOVARIANCE "rviz/PoseWithCovariance"
#define RVIZ_DISPLAY_RANGE              "rviz/Range"
#define RVIZ_DISPLAY_RELATIVEHUMIDITY   "rviz/RelativeHumidity"
#define RVIZ_DISPLAY_ROBOTMODEL         "rviz/RobotModel"
#define RVIZ_DISPLAY_TF                 "rviz/TF"
#define RVIZ_DISPLAY_TEMPERATURE        "rviz/Temperature"
#define RVIZ_DISPLAY_WRENCHSTAMPED      "rviz/WrenchStamped"

class AppRviz : public QObject
{
    Q_OBJECT
public:
    AppRviz(QVBoxLayout *layout, QString node_name);
    ~AppRviz();

    void displayInit(QString classId, bool enabled, QMap<QString, QVariant> namevalue);
    void displayInit(QString classId, QString name, bool enabled, QMap<QString, QVariant> namevalue);

    void removeDisplay(QString name);
    void removeDisplay(QString classId, QString name);
    
    void renameDisplay(QString oldname, QString newname);

    void outDisplaySet(QString path);
    void ReadDisplaySet(QString path);

    void createDisplat(QString displayName, QString topicName);
    void setGlobalOptions(QString frameName, QColor backColor, int frameRate);

    void setPos();
    void setGoal();
    void setMoveCamera();

    void sendGoalTopic();
    void getDisplayTreeMode();

signals:
    void returnModeSignal(QAbstractItemModel *model);

private slots:
    void addTool(rviz::Tool*);

private:
    int getDisplayNum(QString classId);
    int getDisplayNum(QString classId, QString name);
    int getDisplayNumName(QString name);

    rviz::RenderPanel *m_render_panel;
    rviz::VisualizationManager *m_manager;
    rviz::DisplayGroup *m_display_group;
    rviz::Tool *m_currect_tool;
    rviz::ToolManager *m_tool_manager;
    QVBoxLayout *layout;
    QString nodename;

    QMap<QString, QVariant> nullmap;
};
#endif
