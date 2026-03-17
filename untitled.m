% 1. 把你要处理的数据集名字放在一个元胞数组里
dataset_names = {'motordata202603171104538v', ...
                 'motordata202603171106079v', ...
                 'motordata202603171106577v', ...
                 'motordata202603171107506v'};

Ts = 400e-6;      % 采样时间 0.4ms
pole_pairs = 7;   % 电机极对数

% 2. 循环批量处理
for i = 1:length(dataset_names)
    % 从工作区获取当前的 table 数据
    curr_name = dataset_names{i};
    data = evalin('base', curr_name); 
    
    % --- 提取并处理输入 u (q轴指令) ---
    u_raw = data.q_axis;
    % 去除常数趋势 (减去均值)
    u_processed = detrend(u_raw, 'constant'); 
    
    % --- 提取并处理输出 y (转速) ---
    theta_e = data.elec_angle;
    theta_m = unwrap(theta_e) / pole_pairs;      % 解包并转为机械角度
    speed_raw = gradient(theta_m) / Ts;          % 用 gradient 算速度，无延迟且长度不变
    % 去除常数趋势 (减去均值)
    y_processed = detrend(speed_raw, 'constant');
    
    % --- 动态生成变量名并保存到工作区 ---
    u_var_name = sprintf('u_%d', i);
    y_var_name = sprintf('y_%d', i);
    
    assignin('base', u_var_name, u_processed);
    assignin('base', y_var_name, y_processed);
    
    fprintf('成功处理第 %d 组数据，生成了 %s 和 %s (来源: %s)\n', i, u_var_name, y_var_name, curr_name);
end

disp('所有数据批量处理完毕！');