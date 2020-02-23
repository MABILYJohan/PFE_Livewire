#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

class Utils
{
public:
    Utils();

    static int randInt(int low, int high);
	static double distance_euclidienne(double x1, double y1, double x2, double y2);
    static double distance_euclidienne(double x1, double y1, double z1,
                                       double x2, double y2, double z2);

    template<typename T>
    static void permut(T& x, T& y)
    {
        T tmpX;
        tmpX = x;
        T tmpY;
        tmpY = y;
        x = tmpY;
        y = tmpX;
    }


    ////////////////////    VECTOR  ////////////////////////////////

    /*--------------------------------------------------------------
     * Supprime tous les doublons dans @vec
     * -----------------------------------------------------------*/
    template<typename T>
    static void suppr_occur(vector<T> &vec)
    {
        vector<T> vecTmp = vec;
        vec.clear();

        for (auto v : vecTmp)
        {
            if ( ! Utils::is_in_vector(vec, v)) {
                vec.push_back(v);
            }
        }
    }

	template<typename T>
    static void insert_elt(vector<T> &vec, T elt, unsigned pos)
    {
        for (unsigned i=0; i<vec.size(); i++)
        {
            if (i==pos)
            {
                unsigned cptB=0;
                typename vector<T>::iterator it;
                for(it = vec.begin(); it!=vec.end(); ++it)
                {
                    if (cptB == pos) {
                        vec.insert(it, elt);
                        return;
                    }
                    cptB++;
                }
            }
        }
    }

	template<typename T>
    static void erase_elt(vector<T> &vec, T elt)
    {
        unsigned cptA=0;
        for (auto v : vec)
        {
            if (v == elt)
            {
                unsigned cptB=0;
                typename vector<T>::iterator it;
                for(it = vec.begin(); it!=vec.end(); ++it)
                {
                    if (cptB == cptA) {
                        vec.erase(it);
                        return;
                    }
                    cptB++;
                }
            }
            cptA++;
        }
    }

    /*--------------------------------------------------------------
     * @id pour connaître l'emplacement dans @vec
     * -----------------------------------------------------------*/
    template<typename T>
    static T get_min(vector<T> vec, unsigned &id)
    {
        if (vec.empty()) {
            cout << "in " << __FUNCTION__ << ": @vec is empty";
            exit (1);
        }
        T min = vec[0];
        for (unsigned i=0; i<vec.size(); i++) {
            if (min > vec[i]) {
                min = vec[i];
                id = i;
            }
        }
        return min;
    }
    template<typename T>
    static T get_min(vector<T> vec)
    {
        if (vec.empty()) {
            cout << "in " << __FUNCTION__ << ": @vec is empty";
            exit (1);
        }
        T min = vec[0];
        for (auto i : vec) {
            if (min > i) {
                min = i;
            }
        }
        return min;
    }

    template<typename T>
    static bool is_in_vector(vector<T> vec, T val)
    {
        for (auto v : vec)
        {
            if (val == v) return true;
        }
        return false;
    }

    template<typename T>
    static vector<T> invert_vector(vector<T> vec)
    {
        if (vec.empty())    return vec;
        vector<T> myVec;
        int cpt=vec.size()-1;
        do
        {
            myVec.push_back(vec[cpt]);
            cpt--;

        } while(cpt>=0);
        return myVec;
    }

    template<typename T>
    static int get_idVal_in_vector(vector<T> vec, T val)
    {
        for (unsigned i=0; i<vec.size(); i++) {
            if (val == vec[i])  return i;
        }
        return -1;
    }


    ////////////////////    CIRCULAR VECTOR  ////////////////////////////////

    /*--------------------------------------------------------------
     * Vrai si @val2 est à la suite de @val1 dans le
     * tableau @vec .
     * Est circulaire. (ex: vec[0] peut être suivant de vec[fin])
     * -----------------------------------------------------------*/
    template<typename T>
    static bool is_next_of(vector<T> vec, T val1, T val2)
    {
        if (!Utils::is_in_vector(vec, val1)
                || !Utils::is_in_vector(vec, val2)) {
            cout << "in " << __FUNCTION__ << ": not in vector";
            return false;
        }
        unsigned id1;
        for (unsigned i=0; i<vec.size(); i++)
        {
            if (vec[i] == val1) {
                id1 = i;
            }
        }
        // cout << "in " << __FUNCTION__ << ": id1 = " << id1 << endl;
        if (id1 == vec.size()-1   &&  vec[0] == val2) return true;
        if (id1 == vec.size()-1)    return false;
        if (vec[id1+1] == val2) return true;

        return false;
    }

};

#endif // UTILS_H
