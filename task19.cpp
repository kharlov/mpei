#include "plant.h"
#include <cmath>
#include <vector>
#include <iostream>
#include <iomanip>
using namespace std;

// Глобальные переменные
Plant plant;      // модель объекта
int in_channel;   // номер опрашиваемого датчика (от 1 до 6 и от 11 до 90)
int kChannels;    // число каналов управления
vector<int>    L; // номера каналов управления
vector<double> U; // начальные значения управляющих воздействий
double step;      // величина шага поиска
int nStep;        // число шагов поиска

//----------------------------------------------------------------------
void inputData()
{
  // Получение экспериментальных данных.
  // M - номер опрашиваемого датчика (in_channel),
  // K - число каналов управления (kChannels),
  // L1, L2, ..., Lk - номера каналов управления,
  // D - величина шага поиска (step),
  // U1, U2, ..., Uk - начальные значения управляющих воздействий,
  // N - число шагов поиска (nStep).
  
  cout << "Input sensor number M (1-6 or 11-90) : ";
  cin >> in_channel ;

  cout << "Input number of control channels K (1-4) : ";
  cin >> kChannels;

  L.resize(kChannels); // изменить число каналов управления
  cout << "Input " << kChannels << " control channel number L (7-10): ";
  for (size_t i = 0; i < kChannels; i++) {
    cin >> L[i];
  }

  cout << "Input step D: ";
  cin >> step;
  
  U.resize(kChannels); // изменить число управляющих воздействий
  cout << "Input " << kChannels << " initial control values U: ";
  for (size_t i = 0; i < kChannels; i++) {
    cin >> U[i];
  }

  cout << "Input number of steps N: ";
  cin >> nStep;  
  
}
//----------------------------------------------------------------------
double derivative(double y1, double y2, double d)
{
  // вычисление производной для двух значений датчика y1 и y2
  // при шаге управляющего воздействия d
  
  return (y2-y1)/d;
}
//----------------------------------------------------------------------

int main()
{
  plant_init(plant);

  inputData();

  vector<double> P(kChannels); // вектор производных
  
  double ys1;         // измеренное значение датчика при начальной управляющем воздействии
  double ys2;         // измеренное значение датчика после изменения управляющего воздействия
  double value;       // величина управляющего воздействия
  int    out_channel; // номер управляющего канала
  double corr;        // коррекция Q

  size_t i;
  for (i = 0; i < kChannels; i++) {
    P[i] = 1;
  }

  cout << std::setprecision(3);
  cout << endl << "  N | ";
  for (i=0; i<kChannels; i++)
    cout << "  U"<<i << "    | ";

  cout << "   y    |";

  for (i=0; i<kChannels; i++)
    cout << "   P"<<i << "    |";
  cout << "   Q     |" << endl;

  // Нарисовать верхнюю линию таблицы в зависимости от числа управляющих каналов
  cout << "----+";
  for (i=0; i<kChannels; i++)
    cout << "---------+";
  cout << "---------+";
  for (i=0; i<kChannels; i++)
    cout << "---------+";
  cout << "---------+\n";

  for (int iStep=0; iStep<nStep; iStep++) { // цикл по шагам поиска

    double sumP2 = 0; // суммы квадратов производных
    for (size_t i = 0; i < kChannels; i++) {
      value = U[i]; // управляющее воздействие по каналу i
      out_channel = L[i]; // номер управляющего канала
      plant_control(out_channel, value, plant); // подача управляющего воздействия
      ys1 = plant_measure(in_channel, plant); // опрос датчика

      value += step;
      plant_control(out_channel, value, plant); // подача управляющего воздействия
      ys2 = plant_measure(in_channel, plant); // опрос датчика

      P[i] = derivative(ys1,ys2,step);
      sumP2 += P[i]*P[i];
    }
    corr = step / sqrt(sumP2);
    for (size_t i = 0; i < kChannels; i++) {
      U[i] += corr*P[i];
    }

    cout << setw(3) << iStep << " | ";
    for (size_t i = 0; i < kChannels; i++) {
      cout << setw(7) << U[i] << " | ";
    }
    cout << setw(7) << ys1 << " | ";
    for (size_t i = 0; i < kChannels; i++) {
      cout << setw(7) << P[i] << " | ";
    }
    cout << setw(7) << corr << " |\n";

  }

  // Нарисовать нижнюю линию таблицы в зависимости от числа управляющих каналов
  cout << "----+";
  for (i=0; i<kChannels; i++)
    cout << "---------+";
  cout << "---------+";
  for (i=0; i<kChannels; i++)
    cout << "---------+";
  cout << "---------+\n";
}
