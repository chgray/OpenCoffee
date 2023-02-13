# use gnuplot -p ./CoffeePlot.gnuplot
set datafile separator ','
plot 'StorePID.csv' using 0:3 with lines, 'StorePID.csv' using 0:2 with lines
plot 'StoreSteamPID.csv' using 0:3 with lines, 'StoreSteamPID.csv' using 0:2 with lines
plot 'StoreSteamPID.csv' using 0:3 with lines, 'StorePID.csv' using 0:3 with lines