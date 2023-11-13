#include <QtTest>
#include "MapManager.h"

// add necessary includes here

#define TEST_RUN(ix, iy) \
{   \
    int ix1, iy1;   \
    double dx, dy;  \
    m_mapMgr.MapToWorld(ix, iy, dx, dy); \
    m_mapMgr.WorldToMap(dx, dy, ix1, iy1); \
    QCOMPARE(ix1, ix); \
    QCOMPARE(iy1, iy); \
}

class CMapManagerTest : public QObject
{
    Q_OBJECT

public:
    CMapManagerTest(){};
    ~CMapManagerTest(){};

private slots:
    //void test_WorldMap_data();
    void test_WorldMapSmall();
    void test_WorldMapLarge();
    void test_WorldMapWOddHEven();
    void test_WorldMapWOddHOdd();
    void test_WorldMapWEvenHOdd();

};

//准备数据
//void CMapManagerTest::test_WorldMap_data()
//{
//    //插入列
//    QTest::addColumn<int>("ix");
//    QTest::addColumn<int>("iy");
//    QTest::addColumn<double>("dx");
//    QTest::addColumn<double>("dy");

//    //插入行数据
//    double dx, dy;
//    int i,j;
//    i = j = 0;
//    m_mapMgr.MapToWorld(i, j, dx, dy);
//    QTest::addRow("%d,%d",i,j) << i << j << dx << dy;

//    i = 1,j = 1;
//    m_mapMgr.MapToWorld(i, j, dx, dy);
//    QTest::addRow("%d,%d",i,j) << i << j << dx << dy;

//    i = -1,j = -1;
//    m_mapMgr.MapToWorld(i, j, dx, dy);
//    QTest::addRow("%d,%d",i,j) << i << j << dx << dy;

//    i = 199,j = 198;
//    m_mapMgr.MapToWorld(i, j, dx, dy);
//    QTest::addRow("%d,%d",i,j) << i << j << dx << dy;
//}

void CMapManagerTest::test_WorldMapSmall()
{
    CMapManager m_mapMgr;
    TMapInfo m_tInfo;
    m_tInfo.iWidth = 1248;
    m_tInfo.iHight = 672;
    m_tInfo.dResolution = 0.05;
    m_tInfo.dOriginXOffset = -5;
    m_tInfo.dOriginYOffset = -5;
    m_mapMgr.test_SetCurrentMapInfo(m_tInfo);

    int ix1, iy1;
    double dx, dy;
    m_mapMgr.MapToWorld(0, 0, dx, dy);
    m_mapMgr.WorldToMap(-5, -5, ix1, iy1);
    m_mapMgr.MapToWorld(-624, -336, dx, dy);


//    TEST_RUN(0, 0);
//    TEST_RUN(1, 1);
//    TEST_RUN(-1, -1);
//    TEST_RUN(1, -1);
//    TEST_RUN(-1, 1);
}

void CMapManagerTest::test_WorldMapLarge()
{
    CMapManager m_mapMgr;
    TMapInfo m_tInfo;
    m_tInfo.iWidth = 500;
    m_tInfo.iHight = 400;
    m_tInfo.dResolution = 0.05;
    m_tInfo.dOriginXOffset = -5.12;
    m_tInfo.dOriginYOffset = -6.24;
    m_mapMgr.test_SetCurrentMapInfo(m_tInfo);

    TEST_RUN(0, 0);
    TEST_RUN(1, 1);
    TEST_RUN(-1, -1);
    TEST_RUN(1, -1);
    TEST_RUN(-1, 1);

    TEST_RUN(-111, 111);
    TEST_RUN(111, -111);
    TEST_RUN(111, 111);
    TEST_RUN(-111, -111);
}

void CMapManagerTest::test_WorldMapWOddHEven()
{
    CMapManager m_mapMgr;
    TMapInfo m_tInfo;
    m_tInfo.iWidth = 501;
    m_tInfo.iHight = 400;
    m_tInfo.dResolution = 0.05;
    m_tInfo.dOriginXOffset = -5.12;
    m_tInfo.dOriginYOffset = -6.24;
    m_mapMgr.test_SetCurrentMapInfo(m_tInfo);

    TEST_RUN(0, 0);
    TEST_RUN(1, 1);
    TEST_RUN(-1, -1);
    TEST_RUN(1, -1);
    TEST_RUN(-1, 1);

    TEST_RUN(-111, 111);
    TEST_RUN(111, -111);
    TEST_RUN(111, 111);
    TEST_RUN(-111, -111);
}

void CMapManagerTest::test_WorldMapWOddHOdd()
{
    CMapManager m_mapMgr;
    TMapInfo m_tInfo;
    m_tInfo.iWidth = 501;
    m_tInfo.iHight = 401;
    m_tInfo.dResolution = 0.05;
    m_tInfo.dOriginXOffset = -5.12;
    m_tInfo.dOriginYOffset = -6.24;
    m_mapMgr.test_SetCurrentMapInfo(m_tInfo);

    TEST_RUN(0, 0);
    TEST_RUN(1, 1);
    TEST_RUN(-1, -1);
    TEST_RUN(1, -1);
    TEST_RUN(-1, 1);

    TEST_RUN(-111, 111);
    TEST_RUN(111, -111);
    TEST_RUN(111, 111);
    TEST_RUN(-111, -111);
}

void CMapManagerTest::test_WorldMapWEvenHOdd()
{
    CMapManager m_mapMgr;
    TMapInfo m_tInfo;
    m_tInfo.iWidth = 500;
    m_tInfo.iHight = 401;
    m_tInfo.dResolution = 0.05;
    m_tInfo.dOriginXOffset = -5.12;
    m_tInfo.dOriginYOffset = -6.24;
    m_mapMgr.test_SetCurrentMapInfo(m_tInfo);

    TEST_RUN(0, 0);
    TEST_RUN(1, 1);
    TEST_RUN(-1, -1);
    TEST_RUN(1, -1);
    TEST_RUN(-1, 1);

    TEST_RUN(-111, 111);
    TEST_RUN(111, -111);
    TEST_RUN(111, 111);
    TEST_RUN(-111, -111);
}


QTEST_APPLESS_MAIN(CMapManagerTest)

#include "MapManagerTest.moc"
