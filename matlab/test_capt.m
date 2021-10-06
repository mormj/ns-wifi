
y = read_samples_from_file('/tmp/ns_sync_long.fc32',1);

y = reshape(y,64,[]);
y = fftshift(fft(y),1);
y = reshape(y,[],1);

figure; plot(real(y)); hold on; plot(imag(y));

set(gca, 'xlim', [0, 1000])
set(gca, 'ylim', [-.06,.06])