clear all

filename = 'torque_curve.xlsx';
rpm = 2000:500:16000;
trq = xlsread(filename, 'C2:C30');
trq_corrected = trq';
trq_corrected_nm = trq_corrected * 1.3558;

p = polyfit(rpm, trq_corrected_nm, 9);


x1 = linspace(2000,16000);
y1 = polyval(p, x1);


writematrix(p, 'torque_polynomial.csv')
plot(rpm, trq_corrected_nm)
hold on
plot(x1,y1)
hold off