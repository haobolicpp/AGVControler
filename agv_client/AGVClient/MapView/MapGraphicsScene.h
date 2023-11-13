#ifndef CMAPGRAPHICSSCENE_H
#define CMAPGRAPHICSSCENE_H
#include <QGraphicsScene>


class CMapItem;
class CMapGraphicsView;
class CMapGraphicsScene : public QGraphicsScene
{
    Q_OBJECT
public:
    CMapGraphicsScene();

protected:
    //void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
    //void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
    //void mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent) override;

};

#endif // CMAPGRAPHICSSCENE_H
