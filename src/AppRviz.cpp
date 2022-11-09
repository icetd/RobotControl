#include "AppRviz.h"

AppRviz::AppRviz(QVBoxLayout *layout, QString node_name)
{
    nullmap.clear();
    this->layout = layout;
    this->nodename = node_name;
    
    /*create rviz vector*/
    m_render_panel = new rviz::RenderPanel;

    /*add rviz wigget to layout*/
    layout->addWidget(m_render_panel);
    
    /*initialize rviz manager*/
    m_manager = new rviz::VisualizationManager(m_render_panel);
    ROS_ASSERT(m_manager != nullptr);

    /*get display group*/
    m_display_group = m_manager->getRootDisplayGroup();

    /*get current rviz manager tool*/
    m_tool_manager = m_manager->getToolManager();
    ROS_ASSERT(m_tool_manager != nullptr);

    /*initialize camera*/
    m_render_panel->initialize(m_manager->getSceneManager(), m_manager);
    m_manager->initialize();
    m_tool_manager->initialize();
    m_manager->removeAllDisplays();
}

AppRviz::~AppRviz()
{
    if (layout != nullptr && m_render_panel != nullptr) 
        layout->removeWidget(m_render_panel);
    if (m_render_panel != nullptr)
        delete m_render_panel;
    if (m_manager != nullptr)
        delete m_manager;
    if (m_currect_tool != nullptr)
        delete m_currect_tool;
    if (m_tool_manager != nullptr)
        delete m_tool_manager;
    ROS_INFO("RVIZ is shutdown");
}

void AppRviz::displayInit(QString classId, bool enabled, QMap<QString, QVariant> namevalue)
{   
    int num = getDisplayNum(classId);
    if (num == -1) {
        rviz::Display *rvizDisplay = m_manager->createDisplay(classId, classId, true);
        ROS_ASSERT(rvizDisplay != nullptr);
        num = getDisplayNum(classId);
    }
    if (!namevalue.empty()) {
        QMap<QString, QVariant>::iterator it;
        for (it = namevalue.begin(); it != namevalue.end(); it++) {
            m_display_group->getDisplayAt(num)->subProp(it.key())->setValue(it.value());
        }
    }
    m_display_group->getDisplayAt(num)->setEnabled(enabled);
    m_manager->startUpdate();
}
void AppRviz::displayInit(QString classId, QString name, bool enabled, QMap<QString, QVariant> namevalue)
{
    int num = getDisplayNum(classId, name);
    if (num == -1) {
        rviz::Display *rvizDisplay = m_manager->createDisplay(classId, name, true);
        ROS_ASSERT(rvizDisplay != nullptr);
        num = getDisplayNum(classId, name);
    }
    if (!namevalue.empty()) {
        QMap<QString, QVariant>::iterator it;
        for (it = namevalue.begin(); it != namevalue.end(); it++) {
            m_display_group->getDisplayAt(num)->subProp(it.key())->setValue(it.value());
        }
    }
    m_display_group->getDisplayAt(num)->setEnabled(enabled);
    m_manager->startUpdate();
}
void AppRviz::removeDisplay(QString name)
{
    int num = getDisplayNumName(name);
    if (num == -1) 
        return;
    delete m_display_group->getDisplayAt(num);
}

void AppRviz::removeDisplay(QString classId, QString name)
{
    int num = getDisplayNum(classId, name);
    if (num == -1) 
        return;
    delete m_display_group->getDisplayAt(num);
}
void AppRviz::renameDisplay(QString oldname, QString newname)
{
    int num = getDisplayNumName(oldname);
    if (num == -1) 
        return;
    m_display_group->getDisplayAt(num)->setName(newname);
}

/**
 * @brief save the rviz configs to the path
 * 
 * @param path 
 */
void AppRviz::outDisplaySet(QString path)
{
    if (!path.isEmpty()) {
        if (m_manager == nullptr)
            return;
        rviz::Config config;
        m_manager->save(config);
        rviz::YamlConfigWriter yamlconfigwriter;
        yamlconfigwriter.writeFile(config, path);
    }
}
/**
 * @brief load the rviz config from path
 * 
 * @param path 
 */
void AppRviz::ReadDisplaySet(QString path)
{
    if (!path.isEmpty()) {
        if (m_manager == nullptr)
            return;
        rviz::YamlConfigReader yamlconfigreader;
        rviz::Config config;
        yamlconfigreader.readFile(config, path);
        m_manager->load(config);
    }
}

void AppRviz::createDisplat(QString displayName, QString topicName)
{

}
void AppRviz::setGlobalOptions(QString frameName, QColor backColor, int frameRate)
{

}
void AppRviz::setPos()
{

}
void AppRviz::setGoal()
{

}
void AppRviz::setMoveCamera()
{

}

void AppRviz::sendGoalTopic()
{

}

void AppRviz::getDisplayTreeMode()
{
    rviz::PropertyTreeModel *rvizModel = m_manager->getDisplayTreeModel();
    QAbstractItemModel * model = rvizModel;
    emit returnModeSignal(model);
}

void AppRviz::addTool(rviz::Tool *)
{

}

int AppRviz::getDisplayNum(QString classId)
{
    int num = -1;
    for (int i = 0; i < m_display_group->numDisplays(); i++) {
        if (m_display_group->getDisplayAt(i) != nullptr) {
            if (classId == m_display_group->getDisplayAt(i)->getClassId()) {
                num = i;
                break;
            }
        }
    }
    return num;
}

int AppRviz::getDisplayNum(QString classId, QString name)
{
    int num = -1;
    for (int i = 0; i < m_display_group->numDisplays(); i++) {
        if (m_display_group->getDisplayAt(i) != nullptr) {
            if (classId == m_display_group->getDisplayAt(i)->getClassId() && name == m_display_group->getDisplayAt(i)->getName()) {
                num = i;
                break;
            }
        }
    }
    return num;
}

int AppRviz::getDisplayNumName(QString name)
{
    int num = -1;
    for (int i = 0; i < m_display_group->numDisplays(); i++) {
        if (m_display_group->getDisplayAt(i) != nullptr) {
            if (name == m_display_group->getDisplayAt(i)->getName()) {
                num = i;
                break;
            }
        }
 
    }
    return num;
}