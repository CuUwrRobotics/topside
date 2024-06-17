import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mcolors

if self.plot_mode == PlotOption.Spectrogram:

    spectr_data, spectr_extent = Processing.calcSpectrogram(packet)

    # Clear the current axes
    Fig_h.get_axes()[2].cla()

    # Plot the spectrogram
    Fig_h.get_axes()[2].imshow(spectr_data, cmap='magma', origin='lower', aspect='auto', extent=spectr_extent)

    # Normalize the colormap
    norm = mcolors.Normalize(vmin=spectr_data.min(), vmax=spectr_data.max())

    # Add a colorbar to the spectrogram
    Fig_h.colorbar(cm.ScalarMappable(norm=norm, cmap='magma'), ax=Fig_h.get_axes()[2])