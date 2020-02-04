#include "dialoghistogramme.h"


#include <QtWidgets/QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QLegend>
#include <QtCharts/QBarCategoryAxis>
QT_CHARTS_USE_NAMESPACE

#include <QDebug>
#include<cstring>

using namespace std;

DialogHistogramme::DialogHistogramme(QWidget *parent, vector<int> donnees, vector<char*> labels,
                                     char *labelAxe, char *title) :
    QDialog(parent)
{
    setupUi(this);

    this->_donnees = donnees;

    //vector<QBarSet*> sets (donnees.size());
    QBarSet *mySet = new QBarSet("catégories");
    for (unsigned i=0; i<_donnees.size(); i++)
    {
        if (_donnees[i] > 0)
            *mySet << _donnees[i];
    }
    QBarSeries *series = new QBarSeries();
    series->append(mySet);

//    for (unsigned i=0; i<_donnees.size(); i++)
//    {
//        //qDebug() << "labels[" << i << "] = " << labels[i] ;
//        sets[i] = new QBarSet(labels[i]);
//        *sets[i] << donnees[i];
//        series->append(sets[i]);
//    }

    QChart *chart = new QChart();
    chart->addSeries(series);
    chart->setTitle(title);

    QStringList categories;
    for (unsigned i=0; i<labels.size(); i++)
    {
        if (_donnees[i] > 0)
        categories << labels[i];
    }
    QBarCategoryAxis *axisX = new QBarCategoryAxis();
    axisX->append(categories);
    //qDebug() << "labelAxe = " << labelAxe;
    axisX->setTitleText(labelAxe);
    chart->createDefaultAxes();
    chart->setAxisX(axisX, series);


//    /*
//    QBarCategoryAxis *axisY = new QBarCategoryAxis();
//    axisY->setMin("0");
//    axisY->setMax("100");
//    QStringList categories;
//    for (unsigned i=0; i<_donnees.size(); i++)
//    {
//        char truc[20];
//        sprintf(truc, "%d ", i*10);
//        categories << truc;
//    }
//    axisY->append(categories);
//    */


    chart->legend()->setVisible(false);
    //chart->legend()->setAlignment(Qt::AlignBottom);

    _myQChartView->setChart(chart);
    _myQChartView->setRenderHint(QPainter::Antialiasing);

}

