//! Unit tests for bank switching functionality

use crate::common::{Operation, create_mock_driver};
use icm20948::Bank;

#[test]
fn test_bank_switch_basic() {
    let (mut driver, interface) = create_mock_driver();

    // Switch to Bank 1
    driver.select_bank(Bank::Bank1).unwrap();

    // Verify bank switch occurred
    let ops = interface.operations();

    // Should have a two bank switch operations:
    // One when reading whoami in `new`, and a second from the `select_bank` call above
    let bank_switches: Vec<_> = ops
        .iter()
        .filter(|op| matches!(op, Operation::BankSwitch { .. }))
        .collect();

    assert_eq!(bank_switches.len(), 2);
}

#[test]
fn test_bank_switch_noop() {
    let (mut driver, interface) = create_mock_driver();

    // Stay in Bank 0 (current bank)
    driver.select_bank(Bank::Bank0).unwrap();

    let ops = interface.operations();

    // select_bank now always writes REG_BANK_SEL (the cache was removed
    // because generated register accessors can leave it stale). So even
    // "staying" in the same bank produces a write.
    let bank_switches: Vec<_> = ops
        .iter()
        .filter(|op| matches!(op, Operation::BankSwitch { .. }))
        .collect();

    // One from try_new(), one from the explicit select_bank(Bank0) above
    assert_eq!(
        bank_switches.len(),
        2,
        "select_bank always writes REG_BANK_SEL now"
    );
}

#[test]
fn test_bank_switch_failure() {
    let (mut driver, interface) = create_mock_driver();

    // Inject bank switch failure
    interface.fail_bank_switch(true);

    // Attempt to switch banks should fail
    let result = driver.select_bank(Bank::Bank1);
    assert!(result.is_err(), "Bank switch should fail when injected");
}

#[test]
fn test_bank_switch_sequence() {
    let (mut driver, interface) = create_mock_driver();

    // Clear initial operations
    interface.clear_operations();

    // Switch through all banks
    driver.select_bank(Bank::Bank1).unwrap();
    driver.select_bank(Bank::Bank2).unwrap();
    driver.select_bank(Bank::Bank3).unwrap();
    driver.select_bank(Bank::Bank0).unwrap();

    let bank_switch_count = interface.bank_switch_count();

    assert_eq!(bank_switch_count, 4, "Should have 4 bank switches");

    // Verify the sequence
    let ops = interface.operations();
    let bank_switches: Vec<_> = ops
        .iter()
        .filter_map(|op| {
            if let Operation::BankSwitch { from, to } = op {
                Some((from, to))
            } else {
                None
            }
        })
        .collect();

    assert_eq!(bank_switches[0], (&Bank::Bank0, &Bank::Bank1));
    assert_eq!(bank_switches[1], (&Bank::Bank1, &Bank::Bank2));
    assert_eq!(bank_switches[2], (&Bank::Bank2, &Bank::Bank3));
    assert_eq!(bank_switches[3], (&Bank::Bank3, &Bank::Bank0));
}
