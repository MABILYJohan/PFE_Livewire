#ifndef DIALOGHISTOGRAMME_H
#define DIALOGHISTOGRAMME_H

#include <vector>

#include <QtWidgets>
#include <QGraphicsWidget>



#include "ui_dialoghistogramme.h"

using namespace std;

class DialogHistogramme : public QDialog, private Ui::DialogHistogramme
{
    Q_OBJECT

private:
    vector<int> _donnees;

public:
    explicit DialogHistogramme(QWidget *parent, vector<int> _donnees, vector<char *> labels,
                               char *labelAxe, char *title);

};

#endif // DIALOGHISTOGRAMME_H
