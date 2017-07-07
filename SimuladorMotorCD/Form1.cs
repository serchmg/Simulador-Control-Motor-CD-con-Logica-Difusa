using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace SimuladorMotorCD
{
    public struct funcion
    {
        public int type;            //Tipo de funcion de pertenencia
        public bool aplicable;      //Si la regla aplica = 1, si no aplica = 0
        public float limInferior;   //Limite inferior de la funcion
        public float limSuperior;   //Limite superior de la funcion
        public float certeza;       //Valor de certeza de la funcion
    }

    public struct regla
    {
        public int entrada1;        //Entrada 1 de la regla
        public int entrada2;        //Entrada 2 de la regla
        public int salida;          //Salida deseada con las entradas 1 y 2
        public bool valida;         //1 cuando la regla se aplica, 0 cuando la regla no se aplica
    }

    public struct premisa
    {
        public float width;
        public float height;
        public float center;
    }

    public partial class Form1 : Form
    {
        funcion[] entrada1 = new funcion[5];
        funcion[] entrada2 = new funcion[5];
        funcion[] salida = new funcion[5];

        regla[] baseDeReglas = new regla[25];
        regla[] reglasAplicables = new regla[4];

        premisa[] u = new premisa[4];

        double uCrisp;                              //Valor de certeza final
        
        double k1, k2, k3, k4;                      //Valores calculados por RungeKutta
        double[] i = new double[1000];              //Corriente de armadura
        double []Eb = new double[1000];             //Fuerza contraelectromotriz
        double []Tt = new double[1000];             //Par del motor
        double[] w = new double[1000];              //Velocidad angular
        int count;                                  //Numero de ejecucion
        double CO, COSat;                           //Salida de control, salida de control saturada
        double error;                               //Error
        double errorAnt = 0;                        //Error anterior
        double derivadaError = 0;                   //Derivada del error

        public Form1()
        {
            InitializeComponent();
            numericUpDownEa.Value = 0;
            numericUpDownRa.Value = (decimal)0.316;
            numericUpDownLa.Value = (decimal)0.082;
            numericUpDownKt.Value = (decimal)30.2;
            numericUpDownKb.Value = (decimal)317;
            numericUpDownJ.Value = (decimal)0.139;
            numericUpDownB.Value = (decimal)0;
            numericUpDownH.Value = (decimal)0.0005;

            inicializaValores();
            iniciaBaseDeReglas();
        }

        private void iniciaBaseDeReglas()
        {
            baseDeReglas[0].entrada1 = 0;
            baseDeReglas[0].entrada2 = 0;
            baseDeReglas[0].salida = 0;
            baseDeReglas[0].valida = true;

            baseDeReglas[1].entrada1 = 0;
            baseDeReglas[1].entrada2 = 1;
            baseDeReglas[1].salida = 0;
            baseDeReglas[1].valida = true;

            baseDeReglas[2].entrada1 = 0;
            baseDeReglas[2].entrada2 = 2;
            baseDeReglas[2].salida = 0;
            baseDeReglas[2].valida = true;

            baseDeReglas[3].entrada1 = 0;
            baseDeReglas[3].entrada2 = 3;
            baseDeReglas[3].salida = 1;
            baseDeReglas[3].valida = true;

            baseDeReglas[4].entrada1 = 0;
            baseDeReglas[4].entrada2 = 4;
            baseDeReglas[4].salida = 3;
            baseDeReglas[4].valida = true;

            baseDeReglas[5].entrada1 = 1;
            baseDeReglas[5].entrada2 = 0;
            baseDeReglas[5].salida = 0;
            baseDeReglas[5].valida = true;

            baseDeReglas[6].entrada1 = 1;
            baseDeReglas[6].entrada2 = 1;
            baseDeReglas[6].salida = 1;
            baseDeReglas[6].valida = true;

            baseDeReglas[7].entrada1 = 1;
            baseDeReglas[7].entrada2 = 2;
            baseDeReglas[7].salida = 1;
            baseDeReglas[7].valida = true;

            baseDeReglas[8].entrada1 = 1;
            baseDeReglas[8].entrada2 = 3;
            baseDeReglas[8].salida = 2;
            baseDeReglas[8].valida = true;

            baseDeReglas[9].entrada1 = 1;
            baseDeReglas[9].entrada2 = 4;
            baseDeReglas[9].salida = 3;
            baseDeReglas[9].valida = true;

            baseDeReglas[10].entrada1 = 2;
            baseDeReglas[10].entrada2 = 0;
            baseDeReglas[10].salida = 1;
            baseDeReglas[10].valida = true;

            baseDeReglas[11].entrada1 = 2;
            baseDeReglas[11].entrada2 = 1;
            baseDeReglas[11].salida = 2;
            baseDeReglas[11].valida = true;

            baseDeReglas[12].entrada1 = 2;
            baseDeReglas[12].entrada2 = 3;
            baseDeReglas[12].salida = 2;
            baseDeReglas[12].valida = true;

            baseDeReglas[13].entrada1 = 2;
            baseDeReglas[13].entrada2 = 4;
            baseDeReglas[13].salida = 3;
            baseDeReglas[13].valida = true;

            baseDeReglas[14].entrada1 = 3;
            baseDeReglas[14].entrada2 = 0;
            baseDeReglas[14].salida = 1;
            baseDeReglas[14].valida = true;

            baseDeReglas[15].entrada1 = 3;
            baseDeReglas[15].entrada2 = 1;
            baseDeReglas[15].salida = 2;
            baseDeReglas[15].valida = true;

            baseDeReglas[16].entrada1 = 3;
            baseDeReglas[16].entrada2 = 2;
            baseDeReglas[16].salida = 3;
            baseDeReglas[16].valida = true;

            baseDeReglas[17].entrada1 = 3;
            baseDeReglas[17].entrada2 = 3;
            baseDeReglas[17].salida = 3;
            baseDeReglas[17].valida = true;

            baseDeReglas[18].entrada1 = 3;
            baseDeReglas[18].entrada2 = 4;
            baseDeReglas[18].salida = 4;
            baseDeReglas[18].valida = true;

            baseDeReglas[19].entrada1 = 4;
            baseDeReglas[19].entrada2 = 0;
            baseDeReglas[19].salida = 1;
            baseDeReglas[19].valida = true;

            baseDeReglas[20].entrada1 = 4;
            baseDeReglas[20].entrada2 = 1;
            baseDeReglas[20].salida = 3;
            baseDeReglas[20].valida = true;

            baseDeReglas[21].entrada1 = 4;
            baseDeReglas[21].entrada2 = 2;
            baseDeReglas[21].salida = 4;
            baseDeReglas[21].valida = true;

            baseDeReglas[22].entrada1 = 4;
            baseDeReglas[22].entrada2 = 3;
            baseDeReglas[22].salida = 4;
            baseDeReglas[22].valida = true;

            baseDeReglas[23].entrada1 = 4;
            baseDeReglas[23].entrada2 = 4;
            baseDeReglas[23].salida = 4;
            baseDeReglas[23].valida = true;

            baseDeReglas[24].entrada1 = 2;
            baseDeReglas[24].entrada2 = 2;
            baseDeReglas[24].salida = 2;
            baseDeReglas[24].valida = true;
        }

        private void inicializaValores()
        {
            //Limites de Funciones de entrada1
            entrada1[0].limInferior = -12000;
            entrada1[0].limSuperior = -4000;

            entrada1[1].limInferior = -8000;
            entrada1[1].limSuperior = 0;

            entrada1[2].limInferior = -4000;
            entrada1[2].limSuperior = 4000;

            entrada1[3].limInferior = 0;
            entrada1[3].limSuperior = 8000;

            entrada1[4].limInferior = 4000;
            entrada1[4].limSuperior = 12000;

            //Limites de funciones de entrada2
            entrada2[0].limInferior = -12000;
            entrada2[0].limSuperior = -4000;

            entrada2[1].limInferior = -8000;
            entrada2[1].limSuperior = 0;

            entrada2[2].limInferior = -4000;
            entrada2[2].limSuperior = 4000;

            entrada2[3].limInferior = 0;
            entrada2[3].limSuperior = 8000;

            entrada2[4].limInferior = 4000;
            entrada2[4].limSuperior = 12000;

            //Limites de funciones de salida
            salida[0].limInferior = -12;
            salida[0].limSuperior = -4;

            salida[1].limInferior = -8;
            salida[1].limSuperior = 0;

            salida[2].limInferior = -4;
            salida[2].limSuperior = 4;

            salida[3].limInferior = 0;
            salida[3].limSuperior = 8;

            salida[4].limInferior = 4;
            salida[4].limSuperior = 12;
        }

        private void evaluaFunciones()
        {
            int i;
            float centro;

            double sp = (double)numericUpDownSp.Value * (2 * Math.PI) / 60;
            double pv = w[999];

            error = sp - pv;
            derivadaError = (error - errorAnt);

            float inputVal1 = (float)error;
            float inputVal2 = (float)derivadaError;

            //float inputVal1 = (float)numericUpDown1.Value; // error
            //float inputVal2 = (float)numericUpDown2.Value; // derivada del error

            //Evaluacion de las funciones de entrada para el Set Entradas1

            for (i = 0; i < entrada1.Length; i++)
            {
                if (entrada1[i].type == 0)
                {
                    centro = (entrada1[i].limInferior + entrada1[i].limSuperior) / 2;
                    if (inputVal1 > entrada1[i].limInferior && inputVal1 < entrada1[i].limSuperior)
                    {
                        entrada1[i].aplicable = true;
                        if (inputVal1 < centro)
                        {
                            entrada1[i].certeza = (inputVal1 - entrada1[i].limInferior) / (centro - entrada1[i].limInferior);
                        }
                        else if (inputVal1 >= centro)
                        {
                            entrada1[i].certeza = (inputVal1 - entrada1[i].limSuperior) / (centro - entrada1[i].limSuperior);
                        }
                    }
                }
            }

            //Evaluacion de las funciones de entrada para el Set Entradas2
            for (i = 0; i < entrada2.Length; i++)
            {
                if (entrada2[i].type == 0)
                {
                    centro = (entrada2[i].limInferior + entrada2[i].limSuperior) / 2;
                    if (inputVal2 > entrada2[i].limInferior && inputVal2 < entrada2[i].limSuperior)
                    {
                        entrada2[i].aplicable = true;
                        if (inputVal2 < centro)
                        {
                            entrada2[i].certeza = (inputVal2 - entrada2[i].limInferior) / (centro - entrada2[i].limInferior);
                        }
                        else if (inputVal2 >= centro)
                        {
                            entrada2[i].certeza = (inputVal2 - entrada2[i].limSuperior) / (centro - entrada2[i].limSuperior);
                        }
                    }
                }
            }
        }

        private void obtenerReglasAplicables()
        {
            int i;
            int n;
            int[] input1ID = new int[2];
            int[] input2ID = new int[2];

            input1ID[0] = entrada1.Length;
            input1ID[1] = entrada1.Length;

            input2ID[0] = entrada2.Length;
            input2ID[1] = entrada2.Length;

            for (i = 0; i < entrada1.Length; i++)
            {
                entrada1[i].aplicable = false;
            }

            for (i = 0; i < entrada2.Length; i++)
            {
                entrada2[i].aplicable = false;
            }

            //Evaluacion de funciones aplicables y obtencion de grado de certeza
            evaluaFunciones();

            //Identificacion de IDs de funciones aplicables del set entradas1
            n = 0;
            for (i = 0; i < entrada1.Length; i++)
            {
                if (entrada1[i].aplicable)
                {
                    input1ID[n] = i;
                    n++;
                }
            }

            //Identificacion de IDs de funciones aplicables del set entradas2
            n = 0;
            for (i = 0; i < entrada2.Length; i++)
            {
                if (entrada2[i].aplicable)
                {
                    input2ID[n] = i;
                    n++;
                }
            }

            //Limpieza de reglas aplicables en procesos anteriores
            for (i = 0; i < reglasAplicables.Length; i++)
            {
                reglasAplicables[i].valida = false;
            }

            //Identificacion de reglas aplicables
            n = 0;
            for (i = 0; i < baseDeReglas.Length; i++)
            {
                if (baseDeReglas[i].entrada1 == input1ID[0] || baseDeReglas[i].entrada1 == input1ID[1])
                {
                    if (baseDeReglas[i].entrada2 == input2ID[0] || baseDeReglas[i].entrada2 == input2ID[1])
                    {
                        reglasAplicables[n] = baseDeReglas[i];
                        reglasAplicables[n].valida = true;
                        n++;
                    }
                }
            }
        }

        private void fuzzyLogic()
        { 
            float variable1, variable2;
            double numerador, denominador;

            obtenerReglasAplicables();

            numerador = 0;
            denominador = 0;
            
            for (int i = 0; i < reglasAplicables.Length ; i++)
            {
                if (reglasAplicables[i].valida == true)
                {
                    variable1 = entrada1[reglasAplicables[i].entrada1].certeza;
                    variable2 = entrada2[reglasAplicables[i].entrada2].certeza;

                    if (variable1 < variable2)
                    {
                        u[i].height = variable1;
                    }
                    else
                    {
                        u[i].height = variable2;
                    }

                    u[i].width = salida[reglasAplicables[i].salida].limSuperior - salida[reglasAplicables[i].salida].limInferior;
                    u[i].center = salida[reglasAplicables[i].salida].limInferior + u[i].width / 2;

                    //textBox1.Text += u[i].center.ToString();

                    numerador += u[i].center * u[i].width * (u[i].height - (Math.Pow(u[i].height, 2) / 2.0));
                    denominador += u[i].width * (u[i].height - (Math.Pow(u[i].height, 2) / 2.0));
                }
            }

            uCrisp = numerador / denominador;

            CO += uCrisp;

            if (CO > 48)
                COSat = 48;
            else if (CO < 0)
                COSat = 0;
            else
                COSat = CO;

            numericUpDownEa.Value = (decimal)COSat;

            CO = COSat;
            errorAnt = error;
        } 

        private void label2_Click(object sender, EventArgs e)
        {

        }

        private void Form1_Load(object sender, EventArgs e)
        {
            i[0] = 0;
            Eb[0] = 0;
            Tt[0] = 0;
            w[0] = 0;
        }

        private void button1_Click(object sender, EventArgs e)
        {
            for (count = 1; count <= 1000; count++)
            {
                calculos();

                if ((count - 1) % 10 == 0)
                    fuzzyLogic();
            }
        }

        private void numericUpDownH_ValueChanged(object sender, EventArgs e)
        {
           
        }

        private void calculos()
        {
            double iAct, iSig;
            double wAct, wSig;
            double x, y;
            
            //Lectura de datos de numericUpDown
            double h = (double)(numericUpDownH.Value);
            double La = (double)numericUpDownLa.Value / 1000;
            double Ea = (double)numericUpDownEa.Value;
            double Ra = (double)numericUpDownRa.Value;
            double kt = (double)numericUpDownKt.Value / 1000;
            double kb = (double)(numericUpDownKb.Value) * (double)(1.0 / 60.0) * (double)(2 * Math.PI);
            kb = 1 / kb;
            double J = (double)numericUpDownJ.Value / 1000;
            double B = (double)numericUpDownB.Value;
            double load = (double)numericUpDownLoad.Value;

            //Calculo de corriente por runge kutta
            iAct = i[999];
            k1 = h * (1 / La) * (Ea - (Ra * i[999]) - Eb[999]);
            k2 = h * (1 / La) * (Ea - (Ra * (i[999] + k1 / 2)) - Eb[999]);
            k3 = h * (1 / La) * (Ea - (Ra * (i[999] + k2 / 2)) - Eb[999]);
            k4 = h * (1 / La) * (Ea - (Ra * (i[999] + k3)) - Eb[999]);
            iSig = iAct + (1.0/6.0)*(k1 + (2*k2) + (2*k3) + k4);

            //calculo de torque de motor
            Tt[999] = iAct * kt;

            //calculo de velocidad angular por runge kutta
            wAct = w[999-1];
            k1 = h * (1 / J) * (Tt[999] - (B * w[999]) - load);
            k2 = h * (1 / J) * (Tt[999] - (B * (w[999] + k1 / 2)) - load);
            k3 = h * (1 / J) * (Tt[999] - (B * (w[999] + k2 / 2)) - load);
            k4 = h * (1 / J) * (Tt[999] - (B * (w[999] + k3)) - load);
            wSig = wAct + (1.0 / 6.0) * (k1 + (2 * k2) + (2 * k3) + k4);
            w[999] = wSig;

            //Calculo de FEM
            Eb[999] = (kb * wAct);

            for (int j = 0; j < 999; j++)
            {
                i[j] = i[j + 1];
                w[j] = w[j + 1];
            }
            i[999] = iSig;
            w[999] = wSig;

            impresionResultados();

            /*listBox1.Items.Add(h*count
                                + "\t" + Ea + "V"
                                + "\t" + iAct + "A"
                                + "\t" + Ra + "ohm"
                                + "\t" + La + "mH"
                                + "\t" + Eb[999] + "V"
                                + "\t" + Tt[999] + "Nm"
                                + "\t" + kt + "Nm/A"
                                + "\t" + kb + "V/rad/s"
                                + "\t" + wAct + "rad/s" 
                                );
             */
 
            listBox1.Items.Add(h*count
                                + "\t" + Ea + "V"
                                + "\t \t" + error
                                + "\t \t" + CO);
        }

        private void impresionResultados ()
        {
            double corriente;
            double velAngular;
            double setPoint;
            double t;

            chart1.Series["Corriente (A)"].Points.Clear();
            chart2.Series["Velocidad Angular (RPM)"].Points.Clear();
            chart2.Series["SetPoint"].Points.Clear();

            setPoint = (double)numericUpDownSp.Value;

            for (int k = 0; k < 1000; k++)
            {
                t = (double)(numericUpDownH.Value * (k-999+count));
                corriente = i[k];
                velAngular = w[k] * 60 / (2 * Math.PI);
                
                chart1.Series["Corriente (A)"].Points.AddXY(t, corriente);
                chart2.Series["Velocidad Angular (RPM)"].Points.AddXY(t, velAngular);
                chart2.Series["SetPoint"].Points.AddXY(t, setPoint);
            }
        }

        private void buttonRestart_Click(object sender, EventArgs e)
        {
            count = 0;
            chart1.Series["Corriente (A)"].Points.Clear();
            chart2.Series["Velocidad Angular (RPM)"].Points.Clear();
            chart2.Series["SetPoint"].Points.Clear();

            listBox1.Items.Clear();

            numericUpDownEa.Value = 0;

            for (int k = 0; k < 1000; k++)
            {
                i[k] = 0;
                w[k] = 0;
                Eb[k] = 0;
                Tt[k] = 0;
            }

            errorAnt = 0;
            derivadaError = 0;
            CO = 0;
        }

        private void chart1_Click(object sender, EventArgs e)
        {

        }

        private void chart2_Click(object sender, EventArgs e)
        {

        }
    }
}