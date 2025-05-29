import fuzzylite as fl

# === Fuzzy Engine ===
def fuzzy(error):
    engine = fl.Engine(name='LineFollowerFuzzyControl')

    # Line Error: -1.0 (very left) to 1.0 (very right)
    line_error = fl.InputVariable(
        name='Line_Error', minimum=-1.0, maximum=1.0,
        terms=[
            fl.Trapezoid('CENTER', -0.05, -0.05, 0.05, 0.05),
            fl.Trapezoid('SLIGHTLY_LEFT', -0.55, -0.3, -0.2, -0.05),
            fl.Triangle('LEFT', -0.9, -0.55, -0.1),
            fl.Triangle('RIGHT', 0.1, 0.55, 0.9),
            fl.Trapezoid('SLIGHTLY_RIGHT', 0.05, 0.2, 0.3, 0.6),
            fl.Trapezoid('STRONG_LEFT', -1.0, -1.0, -0.9, -0.15),
            fl.Trapezoid('STRONG_RIGHT', 0.15, 0.9, 1.0, 1.0)
        ]
    )

    engine.input_variables = [line_error]#, linear_speed]

    # === Output Variable (TSK) ===
    tsk_control = fl.OutputVariable()
    tsk_control.name = "tskLineFollowerControl"
    tsk_control.enabled = True
    tsk_control.range = (-1.2, 1.2)
    tsk_control.lock_range = True
    tsk_control.defuzzifier = fl.WeightedAverage("TakagiSugeno")
    tsk_control.aggregation = None
    tsk_control.default_value = fl.nan
    tsk_control.lock_previous = False

    # TSK Constants (output terms)
    tsk_control.terms.append(fl.Constant("NO_TURN", 0.0))
    tsk_control.terms.append(fl.Constant("SLIGHTLY_LEFT", 0.05))
    tsk_control.terms.append(fl.Constant("SLIGHTLY_RIGHT", -0.05))
    tsk_control.terms.append(fl.Constant("RIGHT", -0.2))
    tsk_control.terms.append(fl.Constant("LEFT", 0.2))
    tsk_control.terms.append(fl.Constant("STRONG_LEFT", 1.0))
    tsk_control.terms.append(fl.Constant("STRONG_RIGHT", -1.0))

    engine.output_variables = [tsk_control]

    # === TSK Rule Block ===
    tsk_rules = fl.RuleBlock()
    tsk_rules.name = "takagiSugenoControl"
    tsk_rules.enabled = True
    tsk_rules.conjunction = fl.AlgebraicProduct()
    tsk_rules.disjunction = fl.AlgebraicSum()
    tsk_rules.activation = fl.General()
    tsk_rules.implication = None

    # Rules for SLOW speed only
    tsk_rules.rules = [
        fl.Rule.create("if Line_Error is CENTER then tskLineFollowerControl is NO_TURN", engine),
        fl.Rule.create("if Line_Error is SLIGHTLY_LEFT then tskLineFollowerControl is SLIGHTLY_LEFT", engine),
        fl.Rule.create("if Line_Error is SLIGHTLY_RIGHT then tskLineFollowerControl is SLIGHTLY_RIGHT", engine),
        fl.Rule.create("if Line_Error is LEFT then tskLineFollowerControl is LEFT", engine),
        fl.Rule.create("if Line_Error is RIGHT then tskLineFollowerControl is RIGHT", engine),
        fl.Rule.create("if Line_Error is STRONG_LEFT then tskLineFollowerControl is STRONG_LEFT", engine),
        fl.Rule.create("if Line_Error is STRONG_RIGHT then tskLineFollowerControl is STRONG_RIGHT", engine)
    ]

    engine.rule_blocks.append(tsk_rules)

    # === Test with valid input values ===
    engine.input_variables[0].value = fl.scalar(error)  # SLIGHTLY_RIGHT
    engine.process()

    # === Output result ===
    print("TSK Output (Angular Velocity):", fl.Op.str(tsk_control.value))
    # devolver solo 2 decimales
    tsk_control.value = round(tsk_control.value, 2)  # Round to 2 decimal places
    return tsk_control.value


fuzzy(0.1)

