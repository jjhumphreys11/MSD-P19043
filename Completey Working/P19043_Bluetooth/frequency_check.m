data_tremor = readtable('temp_data_patient_1.csv', 'HeaderLines', 1);
data_no_tremor = readtable('temp_data_patient_3.csv', 'HeaderLines', 1);

trem = table2array(data_tremor);
no_trem = table2array(data_no_tremor);

figure(1);
cwt(trem(:,2), 91)
hold off
figure(2);
cwt(no_trem(:,2), 91)