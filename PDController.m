function tau = PDController(t, q, v, controller)
%PDCONTROLLER Computes torques
%   
q_error = controller.q_target - q;
v_error = controller.v_target - v;

%controller.q_error_past = [q_error, controller.q_error_past];
%controller.v_error_past = [v_error, controller.v_error_past];

u = zeros(length(q_error), 1);
if ~isempty(controller.feed_forward)
    u = controller.feed_forward;
end

tau = controller.p.*q_error + controller.d.*v_error + u;
for i = 1:length(tau)
    tau(i) = min(controller.saturation(i), max(tau(i), -controller.saturation(i)));
end
end

