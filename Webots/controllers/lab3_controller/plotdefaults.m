function plotdefaults(fontsize,markersize,linewidth,location)
    set(0,'defaultlinemarkersize',markersize);
    set(0,'defaultlinelinewidth',linewidth);
    set(0,'defaultaxesfontsize',fontsize);
    set(0,'defaulttextinterpreter','latex');
    set(0,'defaultlegendinterpreter','latex')
    set(groot,'defaultaxestickLabelinterpreter','latex')
    set(groot,'defaultlegendlocation',location)
    set(0,'defaultaxesbox','on')
end
