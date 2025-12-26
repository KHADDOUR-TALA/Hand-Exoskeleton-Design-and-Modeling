function createSimulinkGUI()
    fig = figure('Name', 'Simulink Control GUI', 'Position', [100, 100, 900, 600]);
    uicontrol('Style', 'text', 'Position', [20, 550, 100, 22], 'String', 'Theta 1');
    slider1 = uicontrol('Style', 'slider', 'Position', [120, 560, 200, 20], 'Min', -2, 'Max', -1, 'Value', -1.5);
    slider1Value = uicontrol('Style', 'text', 'Position', [330, 550, 50, 22], 'String', num2str(get(slider1, 'Value')));
    uicontrol('Style', 'text', 'Position', [20, 500, 100, 22], 'String', 'Theta 2');
    slider2 = uicontrol('Style', 'slider', 'Position', [120, 510, 200, 20], 'Min', 1, 'Max', 2, 'Value', 1.5);
    slider2Value = uicontrol('Style', 'text', 'Position', [330, 500, 50, 22], 'String', num2str(get(slider2, 'Value')));
    uicontrol('Style', 'text', 'Position', [20, 450, 100, 22], 'String', 'Theta 3');
    slider3 = uicontrol('Style', 'slider', 'Position', [120, 460, 200, 20], 'Min', -1, 'Max', 1, 'Value', 0);
    slider3Value = uicontrol('Style', 'text', 'Position', [330, 450, 50, 22], 'String', num2str(get(slider3, 'Value')));

    uicontrol('Style', 'text', 'Position', [20, 400, 100, 22], 'String', 'Theta 4');
    slider4 = uicontrol('Style', 'slider', 'Position', [120, 410, 200, 20], 'Min', -2, 'Max', 2, 'Value', 0);
    slider4Value = uicontrol('Style', 'text', 'Position', [330, 400, 50, 22], 'String', num2str(get(slider4, 'Value')));
    
    uicontrol('Style', 'pushbutton', 'Position', [350, 550, 150, 22], 'String', 'Start Simulation', ...
        'Callback', @(btn,event) startSimulation(get(slider1, 'Value'), get(slider2, 'Value'), get(slider3, 'Value'), get(slider4, 'Value')));
    
    ax1 = axes('Parent', fig, 'Position', [0.1, 0.3, 0.25, 0.3]);
    xlabel(ax1, 'X-axis 1');
    ylabel(ax1, 'Y-axis 1');
    
    ax2 = axes('Parent', fig, 'Position', [0.4, 0.3, 0.25, 0.3]);
    xlabel(ax2, 'X-axis 2');
    ylabel(ax2, 'Y-axis 2');
    
    ax3 = axes('Parent', fig, 'Position', [0.7, 0.3, 0.25, 0.3]);
    xlabel(ax3, 'X-axis 3');
    ylabel(ax3, 'Y-axis 3');
    
    addlistener(slider1, 'ContinuousValueChange', @(src, event) updateSimulinkVariable('theta1', src.Value, slider1Value));
    addlistener(slider2, 'ContinuousValueChange', @(src, event) updateSimulinkVariable('theta2', src.Value, slider2Value));
    addlistener(slider3, 'ContinuousValueChange', @(src, event) updateSimulinkVariable('theta3', src.Value, slider3Value));
    addlistener(slider4, 'ContinuousValueChange', @(src, event) updateSimulinkVariable('theta4', src.Value, slider4Value));
    
    function updateSimulinkVariable(varName, value, valueDisplay)
        assignin('base', varName, value);
        set(valueDisplay, 'String', num2str(value));
    end

    function startSimulation(theta1, theta2, theta3, theta4)
        assignin('base', 'theta1', theta1);
        assignin('base', 'theta2', theta2);
        assignin('base', 'theta3', theta3);
        assignin('base', 'theta4', theta4);
        simOut = sim('Assem_MCP_PIP');
        
        plot(ax1, simOut.simout1.Time, simOut.simout1.Data);
        plot(ax2, simOut.simout2.Time, simOut.simout2.Data);
        plot(ax3, simOut.simout3.Time, simOut.simout3.Data);
    end
end
