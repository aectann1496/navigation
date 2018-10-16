import bins
from bins.math import grad2rad


settings = bins.Settings(bins.data.Navigation(grad2rad(60), 0, 0, 0),
                              bins.data.Attitude(0, 0, 0, 0, 0),
                              bins.data.Vertical(0, 0),
                              31.25)

runnable = bins.ImmovableModelSupplier(settings.navigation, settings.vertical, 0.001, 100,
                                       bins.Bins(settings,
                                                 bins.PlotConsumer()))
runnable.run()
